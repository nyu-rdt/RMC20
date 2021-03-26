#!/usr/bin/env python

"""
receive_obstacle_data.py

Node to receieve obstacle data from MQTT and publish it to the state code, reference README.
"""

from std_msgs.msg import String
from rdt_localization import Obstacle

import paho.mqtt.client as mqtt
import rospy
import struct

'''
Debug message for when the client connects successfully
'''
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("DEBUG: RECEIVE_OBSTACLE_DATA CONNECTED")

'''
Function called each time a message is received
'''
def on_message(client, userdata, msg):
	# Getting  individual bytes
    data = msg.payload
    front_left = data >> 40
    front_right = (data >> 32) ^ ((data >> 40) << 8)
    front_left_mid = (data >> 24) ^ ((data >> 32) << 8)
    front_right_mid = (data >> 16) ^ ((data >> 24) << 8)
    depo_front = (data >> 8) ^ ((data >> 16) << 8)
    depo_back = data ^ ((data >> 8) << 8)

	# Publishing data to server/obstacle_data
    out_msg = Obstacle()
    out_msg.front_left = front_left
    out_msg.front_right = front_right
    out_msg.front_left_mid = front_left_mid
    out_msg.front_right_mid = front_right_mid
    out_msg.depo_front = depo_front
    out_msg.depo_back = depo_back

    rospy.publish("server/obstacle_data", Obstacle, out_msg)

def main():
    global client

    rospy.init_node("receive_obstacle_data")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883)

    client.subscribe("robotState/obstacleData")

    while not rospy.is_shutdown():
        client.loop()

if __name__ == "__main__":
    main()

