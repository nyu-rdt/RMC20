#!/usr/bin/env python

"""
ping_drive_limb.py

Node to check the connection to the locomotion and limbs subsystems on the robot. The node sends a
special 'ping' packet to both systems, and once they both respond, it publishes a confirmation to
the state controller. This node is dependent on mosquitto.

TODO:
- Make the ping node stop once it confirms both connections
- Make the client only publish two bytes at a time rather than 4
- Once the limbs subsystem is manufactured, make the node check for the limbs connection
"""

from std_msgs.msg import Bool

import rospy
import paho.mqtt.client as clienttype
import struct

client = None # MQTT client
drive_subsystem_connect = False
limbs_subsystem_connect = False

def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected")

'''
Callback function for when input received through MQTT. Parameters client and userdata unused.
'''
def on_message(client, userdata, msg):
    global drive_subsystem_connect
    global limbs_subsystem_connect

    rospy.loginfo("DEBUG: PING RECEIVED BACK")

    # Check which topic the confirmation came in through and confirm the corresponding connection
    if msg.topic == "robotState/drivePing":
        drive_subsystem_connect = True
    elif msg.topic == "robotState/limbPing":
        limbs_subsystem_connect = True

def main():
    global client    

    # Setup ROS node
    rospy.init_node("ping_drive_limb")
    pub = rospy.Publisher('server/ping_drive_limb', Bool, queue_size=10)
    
    # Setup MQTT client
    client = clienttype.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883)

    client.subscribe("robotState/drivePing")
    client.subscribe("robotState/limbPing")
    
    while not rospy.is_shutdown():
        # 252 is the special ping packet. In the current version, the ping is sent as four bytes:
        #   0x00    0x00    0x00    252
        rbt_ping = 252
        rbt_ping = rbt_ping << 24

        client.publish("robotCmds/drive", struct.pack('I', rbt_ping), retain=True)
        client.publish("robotCmds/limbs", struct.pack('I', rbt_ping), retain=True)

        # If the connections are confirmed, publish a confirmation to the state controller
        if drive_subsystem_connect: #and limbs_subsystem_connect:
            pub.publish(True)
        client.loop()
        
if __name__ == '__main__':
    main()