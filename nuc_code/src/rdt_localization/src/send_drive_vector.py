#!/usr/bin/env python

from std_msgs.msg import String
from rdt_localization.msg import Drive_Vector

import paho.mqtt.client as mqtt
import rospy
client = None
# callback function gets executed when connection is made to server
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected: send drive vector")

# callback function gets executed when topic "server/sendDriveVec" receives new drive_vector
def publish_drive_vector(data):
    global client
    robot_spd = int(data.robot_spd)
    offset = int(data.offset_driveMode)
    outdata = bytes([robot_spd,offset])
    client.publish("robotCmds/drive", outdata)

def state_controller_listener():
    global client
    rospy.init_node("send_drive_vector")
    
    client = mqtt.Client()
    client.on_connect = on_connect
    
    # connects to the server on the NUC
    client.connect("localhost", 1883)
    rospy.Subscriber("server/sendDriveVec", String, publish_drive_vector)
    #rospy.spin()
    while not rospy.is_shutdown():
        client.loop()

if __name__ == "__main__":
    state_controller_listener()