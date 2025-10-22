import configparser
import paho.mqtt.client as paho
from paho import mqtt
import rospy
from std_msgs.msg import String


class Bridge():
    def __init__(self):
        self.config = configparser.ConfigParser()
        self.config.read('config.ini')
        self.topic = self.config.get("MQTT", "Topic", fallback="Pickup-Site")
        self._init_rospy()
        self._setupMQTT()

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

        print("connecting to MQTT broker...")
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
        print("Connected with result code " + str(rc))
        self.clientMQTT.subscribe(self.topic)

    def _on_message(self, client, userdata, msg):
        self._rospy_talker(str(msg.topic))


if __name__ == '__main__':
    br = Bridge()
    