#!/usr/bin/env python3
import configparser
import os
import json

import paho.mqtt.client as paho
from paho import mqtt
import rospy
from std_msgs.msg import String


class Bridge():
    def __init__(self):
        self.config = configparser.ConfigParser()

        # Get the absolute path to the config directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'config.ini')
        self.config.read(config_path)

        self.topic = self.config.get("MQTT", "Topic", fallback="Pickup-Site")
        
        # Initialize ROS first
        self._init_rospy()
        
        # Then setup MQTT
        self._setupMQTT()

    def _init_rospy(self):
        rospy.init_node('pickup_site_bridge', anonymous=True)
        
        # Single publisher for cow data
        self.pub_cow = rospy.Publisher('milking/cow_data', String, queue_size=10)
        
        # Legacy publisher for backward compatibility
        self.pub_legacy = rospy.Publisher('calf_num', String, queue_size=10)
        
        self.rate = rospy.Rate(10)  # 10hz
        
        rospy.loginfo("\033[96m📡 ROS publishers initialized:\033[0m")
        rospy.loginfo("\033[96m   - milking/cow_data (JSON)\033[0m")
        rospy.loginfo("\033[96m   - calf_num (legacy)\033[0m")

    def _setupMQTT(self):
        self.clientMQTT = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
        self.clientMQTT.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.clientMQTT.username_pw_set(self.config.get("MQTT", "Username"),
                                        self.config.get("MQTT", "Password"))

        self.clientMQTT.on_connect = self._on_connect
        self.clientMQTT.on_message = self._on_message
        self.clientMQTT.on_subscribe = self._on_subscribe
        self.clientMQTT.on_disconnect = self._on_disconnect

        rospy.loginfo("\033[96m🔌 Connecting to MQTT broker...\033[0m")
        self.clientMQTT.connect(
            self.config.get("MQTT", "Broker"),
            self.config.getint("MQTT", "Port", fallback=8883),
            60)

        self.clientMQTT.loop_start()

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        rospy.loginfo("\033[96mConnected with result code %s\033[0m", str(rc))
        if rc == 0:
            rospy.loginfo("\033[92m✅ Successfully connected to MQTT broker\033[0m")
            
            # Subscribe to single simplified topic with QoS 2
            topic = f"{self.topic}/cow"
            self.clientMQTT.subscribe(topic, qos=2)
            
            rospy.loginfo("\033[96m📬 Subscribed to MQTT topic: %s (QoS: 2)\033[0m", topic)
                
        else:
            rospy.logerr("\033[91m❌ Failed to connect to MQTT broker with code: %s\033[0m", str(rc))

    def _on_subscribe(self, client, userdata, mid, granted_qos, properties=None):
        rospy.loginfo("\033[96m✅ Subscription confirmed with QoS: %s\033[0m", str(granted_qos))

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            rospy.logwarn("\033[93m⚠️  Unexpected MQTT disconnection. Will auto-reconnect. Code: %s\033[0m", str(rc))
        else:
            rospy.loginfo("\033[96m👋 Disconnected from MQTT broker\033[0m")

    def _on_message(self, client, userdata, msg):
        """
        Handle incoming MQTT messages and route them to ROS topics.
        Simplified to handle single topic with JSON payload.
        """
        topic = msg.topic
        payload_raw = msg.payload.decode("utf-8")
        
        try:
            # Parse JSON payload
            payload = json.loads(payload_raw)
            
            cow_num = payload.get("cow_number", "?")
            milk_liters = payload.get("milk_liters", 0.0)
            sequence_id = payload.get("sequence_id", "unknown")
            step = payload.get("step", "?")
            total_steps = payload.get("total_steps", "?")
            
            rospy.loginfo("\033[92m🐄 Cow %s → %.1fL [%s/%s] (Seq: %s)\033[0m",
                         cow_num, milk_liters, step, total_steps, sequence_id)
            
            # Publish to ROS cow data topic (JSON)
            self.pub_cow.publish(payload_raw)
            
            # Publish to legacy topic (just cow number)
            self.pub_legacy.publish(str(cow_num))
            
        except json.JSONDecodeError:
            # Fallback to legacy plain text mode
            rospy.logwarn("\033[93m⚠️  Non-JSON payload (legacy): %s\033[0m", payload_raw)
            self.pub_legacy.publish(payload_raw)




if __name__ == '__main__':
    try:
        br = Bridge()
        rospy.loginfo("\033[96m" + "="*60 + "\033[0m")
        rospy.loginfo("\033[96m🌉 Bridge running - MQTT → ROS\033[0m")
        rospy.loginfo("\033[96m📡 ROS Topics:\033[0m")
        rospy.loginfo("\033[96m   - milking/cow_data (JSON)\033[0m")
        rospy.loginfo("\033[96m   - calf_num (legacy)\033[0m")
        rospy.loginfo("\033[96m" + "="*60 + "\033[0m")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("\033[96m👋 Shutting down bridge\033[0m")
    except KeyboardInterrupt:
        rospy.loginfo("\033[96m👋 Shutting down bridge\033[0m")
