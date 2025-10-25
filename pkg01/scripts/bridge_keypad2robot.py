#!/usr/bin/env python3
import configparser
import os

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
        self._setupMQTT()
        self._init_rospy()

    def _init_rospy(self):
        rospy.init_node('pickup_site', anonymous=True)
        self.pub = rospy.Publisher('calf_num', String, queue_size=10)
        self.rate = rospy.Rate(1) # 1hz

    def _setupMQTT(self):
        self.clientMQTT = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
        self.clientMQTT.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.clientMQTT.username_pw_set(self.config.get("MQTT", "Username"),
                                        self.config.get("MQTT", "Password"))

        self.clientMQTT.on_connect = self._on_connect
        self.clientMQTT.on_message = self._on_message
        self.clientMQTT.on_subscribe = self._on_subscribe
        self.clientMQTT.on_disconnect = self._on_disconnect

        rospy.loginfo("\033[96mConnecting to MQTT broker...\033[0m")
        self.clientMQTT.connect(
            self.config.get("MQTT", "Broker"),
            self.config.getint("MQTT", "Port", fallback=8883),
            60)

        self.clientMQTT.loop_start()

    def _rospy_talker(self, value):
        calf_num = str(value)
        rospy.loginfo(calf_num)
        self.pub.publish(calf_num)

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        rospy.loginfo("\033[96mConnected with result code %s\033[0m", str(rc))
        if rc == 0:
            rospy.loginfo("\033[96mSuccessfully connected to MQTT broker\033[0m")
            self.clientMQTT.subscribe(self.topic)
            rospy.loginfo("\033[96mSubscribed to topic: %s\033[0m", self.topic)
        else:
            rospy.logerr("\033[91mFailed to connect to MQTT broker with code: %s\033[0m", str(rc))

    def _on_subscribe(self, client, userdata, mid, granted_qos, properties=None):
        rospy.loginfo("\033[96mSubscription confirmed with QoS: %s\033[0m", str(granted_qos))

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            rospy.logwarn("\033[93mUnexpected MQTT disconnection. Will auto-reconnect. Code: %s\033[0m", str(rc))
        else:
            rospy.loginfo("\033[96mDisconnected from MQTT broker\033[0m")

    def _on_message(self, client, userdata, msg):
        rospy.loginfo("\033[92mMessage received on topic %s: %s\033[0m", msg.topic, str(msg.payload.decode("utf-8")))
        self._rospy_talker(str(msg.payload.decode("utf-8")))


if __name__ == '__main__':
    try:
        br = Bridge()
        rospy.loginfo("\033[96mBridge node running. Waiting for MQTT messages...\033[0m")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("\033[96mShutting down bridge node\033[0m")
    except KeyboardInterrupt:
        rospy.loginfo("\033[96mShutting down bridge node\033[0m")