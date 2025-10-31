#!/usr/bin/env python3
import configparser
import os
import json
from queue import Queue
from threading import Thread, Lock
import time

import paho.mqtt.client as paho
from paho import mqtt
import rospy
from std_msgs.msg import String

Calf_num = int
Sequence = dict


class Bridge():
    def __init__(self):
        self.config = configparser.ConfigParser()

        # Get the absolute path to the config directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'config.ini')
        self.config.read(config_path)

        self.topic = self.config.get("MQTT", "Topic", fallback="Pickup-Site")
        
        # Queue management (moved from pickup_site)
        # each element is a starting calf number (-1 if base), sequence
        self.sequence_queue: Queue[(Calf_num, Sequence)] = Queue()
        self.current_sequence_id = None
        self.is_processing = False
        self.queue_lock = Lock()

        # sequence that is waiting on the platform
        self.platform_sequece = None

        # dict containing a calf_num and the rest of its sequence (empty dict if ended)
        self.sequences_dict: dict[Calf_num, Sequence] = {}
        
        # Initialize ROS first
        self._init_rospy()
        
        # Then setup MQTT
        self._setupMQTT()
        
        # Start queue processor thread
        self.processor_thread = Thread(target=self._process_queue, daemon=True)
        self.processor_thread.start()
        rospy.loginfo("\033[96m🔄 Queue processor thread started\033[0m")

    def _init_rospy(self):
        rospy.init_node('pickup_site_bridge', anonymous=True)
        
        # # Single publisher for cow data
        # self.pub_cow = rospy.Publisher('milking/cow_data', String, queue_size=10)
        
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
            
            # Subscribe to sequence topic (entire sequences) with QoS 2
            self.clientMQTT.subscribe(self.topic, qos=2)
            
            # Subscribe to all cow topics (wildcard subscription) with QoS 2
            self.clientMQTT.subscribe("cow/#", qos=2)
            
            rospy.loginfo("\033[96m📬 Subscribed to MQTT topics: %s, cow/# (QoS: 2)\033[0m", self.topic)
                
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
        Handle incoming MQTT messages (complete sequences).
        Add them to queue for ordered processing.
        """
        topic = msg.topic
        payload_raw = msg.payload.decode("utf-8")

        rospy.loginfo("\033[96m📨 Received MQTT message on topic: %s\033[0m", topic)
        # rospy.logdebug("\033[90m   Payload: %s\033[0m", payload_raw[:100] + "..." if len(payload_raw) > 100 else payload_raw)

        if topic == "Pickup-Site":
            self._on_platform_message(payload_raw)
        else:
            self._on_calf_message(topic, payload_raw)

    def _on_platform_message(self, payload_raw):
        if self.platform_sequece is not None:
            rospy.logerr("\033[91mError: platform not free\033[0m")
            return
        
        try:
            # Parse JSON payload (entire sequence)
            sequence_data = json.loads(payload_raw)
            
            sequence_id = sequence_data.get("sequence_id", "unknown")
            total_cows = sequence_data.get("total_cows", 0)
            total_liters = sequence_data.get("total_liters", 0.0)
            cows = sequence_data.get("cows", [])
            
            rospy.loginfo("\033[92m📨 Received sequence: %s (%d cows, %.1fL total)\033[0m",
                         sequence_id, total_cows, total_liters)
            
            # save sequence as platform_sequence
            self.platform_sequece = sequence_data
            
            # Add sequence to processing queue
            with self.queue_lock:
                self.sequence_queue.put((-1, sequence_data))
                queue_size = self.sequence_queue.qsize()
            
            rospy.loginfo("\033[96m✅ Sequence queued (Queue size: %d)\033[0m", queue_size)
            
        except json.JSONDecodeError as e:
            rospy.logerr("\033[91m❌ Failed to parse sequence JSON: %s\033[0m", str(e))
        except Exception as e:
            rospy.logerr("\033[91m❌ Error handling message: %s\033[0m", str(e))

    def _on_calf_message(self, topic, payload_raw):
        calf_num = topic.split('/')[-1]
        
        # Check if sequence exists for this calf
        if str(calf_num) not in self.sequences_dict:
            rospy.logwarn("\033[93m⚠️  No sequence found for calf %s - ignoring message\033[0m", calf_num)
            return
        
        next_calf_sequence = self.sequences_dict[str(calf_num)]
        
        # Check if sequence is empty (already completed)
        if not next_calf_sequence.get("cows", []):
            rospy.loginfo("\033[96mℹ️  Sequence for calf %s already completed - ignoring message\033[0m", calf_num)
            return
        
        with self.queue_lock:
            self.sequence_queue.put((calf_num, next_calf_sequence))
            queue_size = self.sequence_queue.qsize()
        
        rospy.loginfo("\033[96m✅ Sequence queued (Queue size: %d)\033[0m", queue_size)
            
    
    def _process_queue(self):
        """
        Background thread that processes sequences from the queue.
        Publishes individual cow messages to ROS topics in order.
        """
        rospy.loginfo("\033[96m🔄 Queue processor thread started\033[0m")
        
        while not rospy.is_shutdown():
            queue_size = self.sequence_queue.qsize()
            #rospy.loginfo("\033[95m🔍 Checking queue (size: %d)...\033[0m", queue_size)
            try:
                # Wait for a sequence from the queue (blocking with timeout)
                try:
                    calf_num_start, sequence_data = self.sequence_queue.get(timeout=1.0)
                except:
                    continue  # Timeout, check again
                
                # Extract metadata
                sequence_id = sequence_data.get("sequence_id", "unknown")
                cows = sequence_data.get("cows", [])
                total_cows = len(cows)
                total_liters = sequence_data.get("total_liters", 0.0)
                
                with self.queue_lock:
                    self.is_processing = True
                    self.current_sequence_id = sequence_id
                
                rospy.loginfo("\033[96m" + "="*70 + "\033[0m")
                rospy.loginfo("\033[92m🚀 Processing sequence: %s\033[0m", sequence_id)
                rospy.loginfo("\033[92m� %d cows - %.1fL total\033[0m", total_cows, total_liters)
                rospy.loginfo("\033[96m" + "="*70 + "\033[0m")

                # create the new sequence
                new_liters = total_liters - cows[0]["liters"]
                new_sequence = {
                    "sequence_id": sequence_id,
                    "cows": cows[1:],
                    "total_cows": len(cows)-1,
                    "total_liters": new_liters
                }
                # save the new sequence in the dict
                self.sequences_dict[str(cows[0]["cow"])] = new_sequence

                # get the ending calf number (-1 if platform)
                calf_num_end = cows[0]["cow"]

                self.pub_legacy.publish(f"{calf_num_start}_{calf_num_end}")

                rospy.loginfo("\033[96m📤 Published: %s → %s\033[0m", calf_num_start, calf_num_end)
                
                # # Process each cow in the sequence
                # for idx, cow_data in enumerate(cows, 1):
                #     cow_num = cow_data.get("cow")
                #     milk_liters = cow_data.get("liters")
                    
                #     rospy.loginfo("\033[92m[%d/%d] Cow %s → %.1fL\033[0m",
                #                  idx, total_cows, cow_num, milk_liters)
                    
                #     # Create payload for ROS
                #     payload = {
                #         "cow_number": cow_num,
                #         "milk_liters": milk_liters,
                #         "sequence_id": sequence_id,
                #         "step": idx,
                #         "total_steps": total_cows
                #     }
                    
                #     payload_json = json.dumps(payload)
                    
                #     # Publish to ROS cow data topic
                #     self.pub_cow.publish(payload_json)
                    
                #     # Publish to legacy topic (just cow number)
                #     self.pub_legacy.publish(str(cow_num))
                    
                #     rospy.loginfo("\033[90m   → Published to ROS topics\033[0m")
                    
                #     # Optional: Add delay between cows if needed
                #     # rospy.sleep(0.5)
                
                rospy.loginfo("\033[96m" + "="*70 + "\033[0m")
                rospy.loginfo("\033[92m✅ Sequence %s completed\033[0m", sequence_id)
                rospy.loginfo("\033[96m" + "="*70 + "\033[0m\n")
                
                with self.queue_lock:
                    self.is_processing = False
                    self.current_sequence_id = None
                
                # Mark task as done
                self.sequence_queue.task_done()

                # if it was a platform task, mark the platform free
                self.platform_sequece = None
                
            except Exception as e:
                rospy.logerr("\033[91m❌ Error processing sequence: %s\033[0m", str(e))
                with self.queue_lock:
                    self.is_processing = False
                    self.current_sequence_id = None
    
    def get_queue_status(self):
        """Return current queue status"""
        with self.queue_lock:
            return {
                "is_processing": self.is_processing,
                "queue_size": self.sequence_queue.qsize(),
                "current_sequence_id": self.current_sequence_id
            }




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
