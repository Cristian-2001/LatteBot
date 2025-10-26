#!/usr/bin/env python3
import configparser
import os
import tkinter as tk
import json
import uuid
from datetime import datetime
from threading import Lock

import paho.mqtt.client as paho
from paho import mqtt

from numerical_keypad import CowMilkInterface

class pickupSite():
    def __init__(self):
        self.root = tk.Tk()
        
        # State management (simplified - no queue here)
        self.queue_lock = Lock()
        self.sequence_counter = 0  # Sequential counter for sequence IDs
        
        # Track all cows that have been used in any started sequence
        self.globally_used_cows = set()  # Set of cow numbers that have been processed
        
        # Create keypad with sequence callback and used_cows_getter
        self.keypad = CowMilkInterface(
            self.root, 
            callback=None,  # Not used anymore
            sequence_callback=self._handle_sequence,
            used_cows_getter=self._get_used_cows  # Function to get globally used cows
        )
        
        self.config = configparser.ConfigParser()
        
        # Get the absolute path to the config directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'config.ini')
        self.config.read(config_path)
        
        self.topic = self.config.get("MQTT", "Topic", fallback="Pickup-Site")
        self._setupMQTT()
    
    def _get_used_cows(self):
        """Return set of cow numbers that have been used in started sequences"""
        with self.queue_lock:
            return self.globally_used_cows.copy()

    def _setupMQTT(self):
        self.clientMQTT = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
        self.clientMQTT.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.clientMQTT.username_pw_set(self.config.get("MQTT", "Username"),
                                        self.config.get("MQTT", "Password"))

        self.clientMQTT.on_connect = self._on_connect

        print("connecting to MQTT broker...")
        self.clientMQTT.connect(
            self.config.get("MQTT", "Broker"),
            self.config.getint("MQTT", "Port", fallback=8883),
            60)

        self.clientMQTT.loop_start()

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        print("Connected with result code " + str(rc))

    def _handle_sequence(self, sequence):
        """
        Handle new sequence from keypad.
        Publishes sequence directly to MQTT - queue management is in bridge.
        sequence: List of tuples [(cow_number, milk_liters), ...]
        """
        # Generate unique sequence ID
        with self.queue_lock:
            self.sequence_counter += 1
            sequence_number = self.sequence_counter
        
        # Create sequence metadata
        timestamp = datetime.now()
        sequence_id = f"SEQ{sequence_number:04d}_{timestamp.strftime('%Y%m%d_%H%M%S')}"
        uuid_id = str(uuid.uuid4())[:8]  # Short UUID for uniqueness
        
        # Complete sequence ID format: SEQ0001_20251025_143022_a3b4c5d6
        full_sequence_id = f"{sequence_id}_{uuid_id}"
        
        # Mark all cows in this sequence as globally used
        cow_numbers = [cow_num for cow_num, _ in sequence]
        with self.queue_lock:
            self.globally_used_cows.update(cow_numbers)
        
        print(f"\n📋 New sequence received: {len(sequence)} cows")
        print(f"🔖 Sequence ID: {full_sequence_id}")
        print(f"🐄 Globally used cows: {sorted(self.globally_used_cows)}")
        
        total_liters = sum(float(liters) for _, liters in sequence)
        print(f"📊 Total liters in sequence: {total_liters:.1f}L")
        
        # Log sequence details
        for idx, (cow_num, liters) in enumerate(sequence, 1):
            print(f"   {idx}. Cow {cow_num} → {liters} liters")
        
        # Add dummy entry to indicate return to base
        print(f"   {len(sequence) + 1}. Return to base (cow_id = -1)")
        
        # Publish entire sequence to MQTT as a single message
        # Bridge will handle queue processing
        # Always append a dummy entry with cow=-1 to indicate return to base
        cows_list = [{"cow": cow, "liters": float(liters)} for cow, liters in sequence]
        cows_list.append({"cow": -1, "liters": 0.0})  # Dummy entry for base return
        
        sequence_payload = {
            "sequence_id": full_sequence_id,
            "sequence_number": sequence_number,
            "timestamp": timestamp.isoformat(),
            "cows": cows_list,
            "total_cows": len(sequence),  # Keep original count (without dummy entry)
            "total_liters": total_liters
        }
        
        topic = f'{self.topic}/sequence'
        self.clientMQTT.publish(
            topic=topic,
            payload=json.dumps(sequence_payload),
            qos=2  # Exactly once delivery
        )
        
        print(f"📡 Published complete sequence to '{topic}'")
        print("="*60 + "\n")
        
        # Force UI update to show newly blocked cows
        self.keypad.update_cow_dropdown_colors()

    def loop(self):
        self.root.bind('q', lambda e: self.root.quit())
        self.root.mainloop()


if __name__ == '__main__':
    pickup_site = pickupSite()
    pickup_site.loop()
