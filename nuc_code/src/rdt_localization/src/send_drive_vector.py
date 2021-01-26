#!/usr/bin/env python

from std_msgs.msg import String
from rdt_localization.msg import Drive_Vector

import paho.mqtt.client as mqtt
import rospy
import struct

client = None
# callback function gets executed when connection is made to server
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected: send drive vector")

# callback function gets executed when topic "server/sendDriveVec" receives new drive_vector
def publish_drive_vector(data):
    global client
    rbt_spd = int(data.robot_spd)
    rbt_offset = int(data.offset_driveMode)
    rbt_spd = rbt_spd << 24
    rbt_offset = rbt_offset << 16
    rbt_data = rbt_spd + rbt_offset
    client.publish("robotCmds/drive", struct.pack('I', rbt_data))

def state_controller_listener():
    global client
    rospy.init_node("send_drive_vector")
    
    client = mqtt.Client()
    client.on_connect = on_connect
    
    # connects to the server on the NUC
    client.connect("localhost", 1883)
    rospy.Subscriber("server/send_drive_vec", Drive_Vector, publish_drive_vector)
    # rbt_spd = 150
    # rbt_offset = 100
    # rbt_spd = rbt_spd << 24
    # rbt_offset = rbt_offset << 16
    # rbt_data = rbt_spd + rbt_offset
    while not rospy.is_shutdown():
        # rospy.loginfo(rbt_data)
        # client.publish("robotCmds/drive", struct.pack('I', rbt_data))
        client.loop()

if __name__ == "__main__":
    state_controller_listener()
