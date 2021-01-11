#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe

client = None

from std_msgs.msg import Bool

drive_subsystem_connect = False
limbs_subsystem_connect = False

def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected")

def drive_ping_received(data):
    global drive_subsystem_connect
    drive_subsystem_connect = True
    
def limbs_ping_received(data):
    global limbs_subsystem_connect
    limbs_subsystem_connect = True

def main():
    global client    
    rospy.init_node("ping_drive_and_limb")
    pub = rospy.Publisher('server/send_drive_vec', Bool, queue_size=10)
    client = mqtt.Client()
        client.on_connect = on_connect
    subscribe.callback(drive_ping_received, "robotState/drivePing", hostname="localhost", port=1883)
    subscribe.callback(limbs_ping_received, "robotState/limbPing", hostname="localhost", port=1883)
    while not rospy.is_shutdown():
        client.publish("robotCmds/drive", 252, retain=True)
        client.publish("robotCmds/limbs", 252, retain=True)
        if drive_subsystem_connect and limbs_subsystem_connect:
            pub.publish(True)
        client.loop()
        
if __name__ == '__main__':
    main()