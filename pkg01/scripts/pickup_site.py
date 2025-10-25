#!/usr/bin/env python3
import configparser
import os
import tkinter as tk
from queue import Queue
from threading import Thread, Lock
import time
import json
import uuid
from datetime import datetime

import paho.mqtt.client as paho
from paho import mqtt

from numerical_keypad import CowMilkInterface

class pickupSite():
    def __init__(self):
        self.root = tk.Tk()
        
        # Create sequence queue and state management
        self.sequence_queue = Queue()
        self.current_sequence = []
        self.current_sequence_id = None
        self.is_processing = False
        self.queue_lock = Lock()
        self.sequence_counter = 0  # Sequential counter for human-readable IDs
        
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
        
        # Start queue processor thread
        self.processor_thread = Thread(target=self._process_queue, daemon=True)
        self.processor_thread.start()
    
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
        
        # Package sequence with metadata
        sequence_data = {
            "id": full_sequence_id,
            "sequence_number": sequence_number,
            "timestamp": timestamp.isoformat(),
            "cows": sequence.copy(),
            "total_cows": len(sequence),
            "total_liters": sum(float(liters) for _, liters in sequence)
        }
        
        with self.queue_lock:
            # Add sequence to queue
            self.sequence_queue.put(sequence_data)
            queue_size = self.sequence_queue.qsize()
            
        print(f"✅ Sequence added to queue (Queue size: {queue_size})")
        print(f"📊 Total liters in sequence: {sequence_data['total_liters']:.1f}L")
        
        # Log sequence details
        for idx, (cow_num, liters) in enumerate(sequence, 1):
            print(f"   {idx}. Cow {cow_num} → {liters} liters")
        
        # Force UI update to show newly blocked cows
        self.keypad.update_cow_dropdown_colors()
    
    def _process_queue(self):
        """
        Background thread that processes sequences from the queue.
        Sends MQTT messages for each cow in sequence.
        """
        print("🔄 Queue processor thread started")
        
        while True:
            try:
                # Wait for a sequence from the queue (blocking)
                sequence_data = self.sequence_queue.get()
                
                # Extract metadata
                sequence_id = sequence_data["id"]
                sequence_number = sequence_data["sequence_number"]
                sequence = sequence_data["cows"]
                total_liters = sequence_data["total_liters"]
                
                with self.queue_lock:
                    self.is_processing = True
                    self.current_sequence = sequence.copy()
                    self.current_sequence_id = sequence_id
                
                print(f"\n🚀 Processing sequence: {sequence_id}")
                print(f"📦 {len(sequence)} cows - {total_liters:.1f}L total")
                print("="*60)
                
                # Process each cow in the sequence
                for idx, (cow_num, liters) in enumerate(sequence, 1):
                    print(f"[{idx}/{len(sequence)}] Cow {cow_num} → {liters}L")
                    
                    # Single MQTT message with all necessary info
                    payload = {
                        "cow_number": cow_num,
                        "milk_liters": float(liters),
                        "sequence_id": sequence_id,
                        "step": idx,
                        "total_steps": len(sequence)
                    }
                    
                    # Publish to single topic
                    topic = f'{self.topic}/cow'
                    self.clientMQTT.publish(
                        topic=topic,
                        payload=json.dumps(payload),
                        qos=2  # Exactly once delivery
                    )
                    print(f"   📡 Published to '{topic}'")
                
                print("="*60)
                print(f"✅ Sequence {sequence_id} completed\n")
                
                with self.queue_lock:
                    self.is_processing = False
                    self.current_sequence = []
                    self.current_sequence_id = None
                
                # Mark task as done
                self.sequence_queue.task_done()
                
            except Exception as e:
                print(f"❌ Error processing sequence: {e}")
                with self.queue_lock:
                    self.is_processing = False
                    self.current_sequence = []
                    self.current_sequence_id = None
    
    def get_queue_status(self):
        """Return current queue status"""
        with self.queue_lock:
            return {
                "is_processing": self.is_processing,
                "queue_size": self.sequence_queue.qsize(),
                "current_sequence": self.current_sequence.copy(),
                "current_sequence_id": self.current_sequence_id,
                "total_sequences_processed": self.sequence_counter
            }

    def loop(self):
        self.root.bind('q', lambda e: self.root.quit())
        self.root.mainloop()


if __name__ == '__main__':
    pickup_site = pickupSite()
    pickup_site.loop()
