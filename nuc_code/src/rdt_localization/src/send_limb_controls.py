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
    global client

    # Drive commands are sent through the MQTT network in the following format:
    #   0x00    0x00    door    linActs_speed   arm_speed   drum_speed

    # encode all 4 fields into 1 byte
    door_b = int(data.door)
    door_b = door << 6 # 0 or 1 --> 00 or 01


    arm_b = int(data.arm_speed)
    arm_b = arm_b << 8 # maybe change this later

    drum_b = int(data.drum_speed)
    drum_b = drum_b << 8 

    linActs_b = int(data.linActs_speed)
    linActs_b = linActs_b << 8

    rbt_data = door_b + arm_b + drum_b + linActs_b # contains 4 fields

    client.publish("robotCmds/limbs", struct.pack('I', rbt_data))

def main():
	# Setup ROS node
    rospy.init_node("send_limb_vector")
    
    # Setup MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect("localhost", 1883)
    
    rospy.Subscriber("server/send_limb_vec", Limb_Vector, publish_limb_vector)

    while not rospy.is_shutdown():
        client.loop()

if __name__ == "__main__":
	main()
