#!/usr/bin/env python3
import configparser
import os
import tkinter as tk

import paho.mqtt.client as paho
from paho import mqtt

from numerical_keypad import NumericalKeypad

class pickupSite():
    def __init__(self):
        self.root = tk.Tk()
        self.keypad = NumericalKeypad(self.root, callback=self._useData)
        self.config = configparser.ConfigParser()
        
        # Get the absolute path to the config directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'config.ini')
        self.config.read(config_path)
        
        self.topic = self.config.get("MQTT", "Topic", fallback="Pickup-Site")
        self._setupMQTT()

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

    def _useData(self, calf_num):
        print("Received: ", calf_num)
        
        # TODO: serve il retain?
        self.clientMQTT.publish(topic=f'{self.topic}', payload=calf_num)

    def loop(self):
        self.root.bind('q', lambda e: self.root.quit())
        self.root.mainloop()


if __name__ == '__main__':
    pickup_site = pickupSite()
    pickup_site.loop()