import configparser
import os
import time

import paho.mqtt.client as paho
from paho import mqtt
import serial
import json
import serial.tools.list_ports


class Bridge():

    def __init__(self):
        self.starting_time = time.time()

        self.config = configparser.ConfigParser()

        # Get the absolute path to the config directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'config.ini')
        self.config.read(config_path)

        self.topic_publish = None   # is a calf number
        self.topic_subscribe = self.config.get("MQTT", "Topic", fallback="Pickup-Site/cow")
        self.setupSerial()
        self.setupMQTT()

        self.calf_limits = {}
        self.starting_weights = {}

    def setupSerial(self):
        """
        This method is used to set up the serial port for communication. It first checks if the configuration file
        specifies to use the port description. If so, it uses the port name specified in the configuration file.
        If not, it lists all available ports and checks each port's description against the port description specified
        in the configuration file. If a match is found, it uses that port.

        Once the port is selected, it attempts to open a connection to the port with a baud rate of 9600 and a timeout of 0.
        If the connection fails, it prints an error message and sets the serial object to None.

        Finally, it initializes an empty list to serve as the internal input buffer from the serial port.
        """
        # open serial port
        self.ser = None

        # self.portname = "COM2"

        if self.config.get("Serial", "UseDescription", fallback=False):
            self.portname = self.config.get("Serial", "PortName", fallback="COM5")
        else:
            print("list of available ports: ")
            ports = serial.tools.list_ports.comports()

            for port in ports:
                print(port.device)
                print(port.description)
                if self.config.get("Serial", "PortDescription", fallback="arduino").lower() \
                        in port.description.lower():
                    self.portname = port.device

        try:
            if self.portname is not None:
                print("connecting to " + self.portname)
                self.ser = serial.Serial(self.portname, 9600, timeout=0)
        except Exception as e:
            print("Error connecting to {}: {}".format(self.portname, str(e)))
            self.ser = None

        # self.ser.open()

        # internal input buffer from serial
        self.inbuffer = []

    def setupMQTT(self):
        self.clientMQTT = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
        self.clientMQTT.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.clientMQTT.username_pw_set(self.config.get("MQTT", "Username"),
                                        self.config.get("MQTT", "Password"))

        self.clientMQTT.on_connect = self._on_connect
        self.clientMQTT.on_subscribe = self._on_subscribe
        self.clientMQTT.on_message = self._on_message

        print("connecting to MQTT broker...")
        self.clientMQTT.connect(
            self.config.get("MQTT", "Broker"),
            self.config.getint("MQTT", "Port", fallback=8883),
            60)

        self.clientMQTT.loop_start()

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        print("Connected with result code " + str(rc))
        if rc == 0:
            print("Successfully connected to MQTT broker")
            self.clientMQTT.subscribe(self.topic_subscribe)
            print("Subscribed to topic:", self.topic_subscribe)
        else:
            print("Failed to connect to MQTT broker with code:", str(rc))

    def _on_message(self, client, userdata, msg):
        print(f"Message received on topic: {msg.topic}")
        print(f"Message payload: {msg.payload.decode()}")

        try:
            data = json.loads(msg.payload.decode())
            cow_numbers = data.get('cows')
            for cow_data in cow_numbers[:-1]:
                calf_id = cow_data['cow']
                limit = cow_data['liters']
            
                if calf_id is not None and limit is not None:
                    self.calf_limits[calf_id] = limit
                    print(f"Updated limits for calf {calf_id}: {limit}")
        except Exception as e:
            print(f"Error parsing message: {e}")

    def _on_subscribe(self, client, userdata, mid, granted_qos, properties=None):
        print("Subscription confirmed with QoS:", str(granted_qos))

    def loop(self):
        # infinite loop for serial managing
        while (True):
            # look for a byte from serial
            if not self.ser is None:

                if self.ser.in_waiting > 0:
                    # data available from the serial port
                    lastchar = self.ser.read(1)

                    if lastchar == b'\xfe':  # EOL
                        print("\nValue received")
                        self.useData()
                        self.inbuffer = []
                    else:
                        # append
                        self.inbuffer.append(lastchar)

    def useData(self):
        val = None

        # I have received a packet from the serial port. I can use it
        if len(self.inbuffer) < 3:  # at least header, size, footer
            return False
        # split parts
        if self.inbuffer[0] != b'\xff':
            return False

        self.topic_publish = int.from_bytes(self.inbuffer[1], byteorder='little')

        weight = int.from_bytes(self.inbuffer[2], byteorder='little')
        print(weight)

        print(self.topic_publish, self.calf_limits)
        if self.topic_publish not in self.calf_limits:
            return

        # if it is the first time it receives a weight from this calf, save it
        if self.topic_publish not in self.starting_weights:
            self.starting_weights[self.topic_publish] = weight
        
        if weight == 0:
            del self.calf_limits[self.topic_publish]
            del self.starting_weights[self.topic_publish]
            return
        if weight <= self.starting_weights[self.topic_publish] - self.calf_limits[self.topic_publish]:
            val = 1
        elif time.time() - self.starting_time > 20:
            val = 0

        if val is not None:
            # TODO: serve il retain?
            # val can be 0 if reached time-out, 1 if the calf finished to drink
            self.clientMQTT.publish(topic=f'{self.topic_publish}', payload=val, retain=True)


if __name__ == '__main__':
    br = Bridge()
    br.loop()
    