#!/usr/bin/env python

from std_msgs.msg import String

import paho.mqtt.client as mqtt
import rospy

# callback function gets executed when connection is made to server
def on_connect(client, userdata, flags, rc):
	rospy.loginfo("Connected: send drive vector")

# callback function gets executed when topic "server/sendDriveVec" receives new drive_vector
def publish_drive_vector(data):
	drive_vector = str(data.data)
	client.publish("robotCmds/drive", drive_vector)

def state_controller_listener():
	rospy.init_node("send_drive_vector")
	rospy.Subscriber("server/send_drive_vec", String, publish_drive_vector)
	
	client = mqtt.Client()
	client.on_connect = on_connect		
	
	# connects to the server on the NUC
	client.connect("localhost", 1883)

	# THIS LINE IS FOR TESTING, DELETE LATER
	for i in range(0, 10):		
		client.publish("robotCmds/drive", "00100000")

	#rospy.spin()
	while not rospy.is_shutdown():
		client.loop()

if __name__ == "__main__":
	state_controller_listener()
