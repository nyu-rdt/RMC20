#!/usr/bin/env python

"""
send_drive_vector.py

Node to relay drive commands from the state controller or the PID controller to the robot through 
the MQTT network. For more information on the format of the drive commands, reference README.
"""

from std_msgs.msg import String
from rdt_localization.msg import Drive_Vector

import paho.mqtt.client as mqtt
import rospy
import struct

client = None

'''
Debug message for when the client connects successfully
'''
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("DEBUG: SEND_DRIVE_VECTOR CONNECTED")

'''
Relay the last received drive command to the robot
data: rdt_localization/Drive_Vector
'''
def publish_drive_vector(data):
    global client

    # Drive commands are sent through the MQTT network in the following format:
    #   0x00    0x00    rbt_offset  rbt_speed
    rbt_spd = int(data.robot_spd)
    rbt_offset = int(data.offset_driveMode)
    rbt_spd = rbt_spd << 24
    rbt_offset = rbt_offset << 16
    rbt_data = rbt_spd + rbt_offset

    client.publish("robotCmds/drive", struct.pack('I', rbt_data))

def main():
    global client

    # Setup ROS node
    rospy.init_node("send_drive_vector")
    
    # Setup MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect("localhost", 1883)
    
    rospy.Subscriber("server/send_drive_vec", Drive_Vector, publish_drive_vector)

    while not rospy.is_shutdown():
        client.loop()

if __name__ == "__main__":
    main()
