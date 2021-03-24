"""
receive_sensor_data.py

Node to receieve sensor data from MQTT and publish it to the state code, reference README.
"""

from std_msgs.msg import String
from rdt_localization import Sensor_Data
import paho.mqtt.client as mqtt
import rospy
import struct

'''
Debug message for when the client connects successfully
'''
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("DEBUG: RECEIVE_SENSOR_DATA CONNECTED")

'''
Function called each time a message is received
'''
def on_message(client, userdata, msg): 
	# Getting 3 bytes and 2 bits
    data = msg.payload
    arm_pos = data >> 16
    lin_act = (data >> 8) ^ (arm_pos << 8)
    drum_cont = (data >> 7) & 1
    drum_turn = (data >> 6) & 1
    
	# Publishing data to server/sensor_data
    out_msg = Sensor_Data()
    out_msg.arm_pos = arm_pos
    out_msg.lin_act = lin_act
    out_msg.drum_cont = drum_cont
    out_msg.drum_turn = drum_turn

    rospy.publish("server/sensor_data", Sensor_Data, out_msg)

def main():
    global client

    rospy.init_node("receive_sensor_data")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883)

    client.subscribe("robotState/sensorData")

    while not rospy.is_shutdown():
        client.loop()

if __name__ == "__main__":
    main()
