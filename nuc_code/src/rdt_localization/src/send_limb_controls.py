#!/usr/bin/env python

from std_msgs.msg import String

import paho.mqtt.client as mqtt
import rospy

# callback function gets executed when connection is made to server
def on_connect(client, userdata, flags, rc):
	rospy.loginfo("Connected, send limb vector")

# callback function gets executed when topic "server/sendLimbVec" receives new limb_vector
def publish_limb_vector(data):
	limb_vector = str(data.data)
	client.publish("robotCmds/limbs", limb_vector)

def state_controller_listener():
	rospy.init_node("send_limb_vector")
	rospy.Subscriber("server/sendLimbVec", String, publish_limb_vector)

	client = mqtt.Client()
	client.on_connect = on_connect
	
	# connects to the server on the NUC
	client.connect("localhost", 1883)

	rospy.spin()

if __name__ == "__main__":
	state_controller_listener()
