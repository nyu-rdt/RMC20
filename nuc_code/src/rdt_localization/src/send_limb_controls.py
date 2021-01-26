#!/usr/bin/env python

"""
send_limb_controls.py

Node to relay limb commands from the state controller to the robot through the MQTT network. For 
more information on the format of the limb commands, reference README.

TODO:
- Implement publish_limb_vector once the subsystem gets manufactured
"""

from std_msgs.msg import String

import paho.mqtt.client as mqtt
import rospy

'''
Debug message for when the client connects successfully
'''
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("DEBUG: SEND_LIMB_VECTOR CONNECTED")

'''
Relay the last received drive command to the robot
data: TBD
'''
def publish_limb_vector(data):
	pass

def main():
	# Setup ROS node
    rospy.init_node("send_limb_vector")
    
    # Setup MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect("localhost", 1883)
    
    rospy.Subscriber("server/send_limb_vec", Drive_Vector, publish_limb_vector)

    while not rospy.is_shutdown():
        client.loop()

if __name__ == "__main__":
	main()
