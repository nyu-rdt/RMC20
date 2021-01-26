#!/usr/bin/env python

from std_msgs.msg import Bool

import rospy
import paho.mqtt.client as clienttype

import struct
client = None

drive_subsystem_connect = False
limbs_subsystem_connect = False

def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected")

def drive_ping_received(data):
    global drive_subsystem_connect
    rospy.loginfo("Drive connected")
    drive_subsystem_connect = True
    
def limbs_ping_received(data):
    global limbs_subsystem_connect
    rospy.loginfo("Limbs connected")
    limbs_subsystem_connect = True

def on_message(client, userdata, msg):
    global drive_subsystem_connect
    global limbs_subsystem_connect
    rospy.loginfo("Message received")
    if msg.topic == "robotState/drivePing":
        drive_subsystem_connect = True
    elif msg.topic == "robotState/limbPing":
        limbs_subsystem_connect = True

def main():
    global client    
    rospy.init_node("ping_drive_limb")
    pub = rospy.Publisher('server/ping_drive_limb', Bool, queue_size=10)
    
    client = clienttype.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883)

    client.subscribe("robotState/drivePing")
    client.subscribe("robotState/limbPing")
    
    #subscribe.callback(drive_ping_received, "robotState/drivePing", hostname="localhost", port=1883)
    #subscribe.callback(limbs_ping_received, "robotState/limbPing", hostname="localhost", port=1883)
    while not rospy.is_shutdown():
        rbt_ping = 252
        rbt_ping = rbt_ping << 24
        client.publish("robotCmds/drive", struct.pack('I', rbt_ping), retain=True)
        client.publish("robotCmds/limbs", struct.pack('I', rbt_ping), retain=True)
        #rospy.loginfo("loop de loop")
        if drive_subsystem_connect: #and limbs_subsystem_connect:
            pub.publish(True)
        client.loop()
        
if __name__ == '__main__':
    main()