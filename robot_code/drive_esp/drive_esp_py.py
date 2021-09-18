#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import KeyManager

#TODO: Establish dummy vars for testing e.g. forward movement

def on_connect(client, userdata, flags, rc): 
    # beep boop connect code (for ROS?)
    return 0 # delete later

# Convert motor values to bytes and publish it to the motors topic
def publish_motors_vector(motor1, motor2, motor3, motor4):
    global client

    # Drive commands are sent through the MQTT network in the following format: 
    #TODO: Add the .msg format for motors
    #   motor1      motor2      motor3      motor4
    
    # encode all 4 fields (MOTORS) into 4 byte 
    rdt_data = bytes([motor1, motor2, motor3, motor4])

    client.publish("robotCmds/motors", rdt_data) #TODO: Create new topic for motors - holds power levels of each motor

#kek there used to be 16 lines of code here but nobody will know what they were

def main():
    global client
    
    key_man = KeyManager()

    # Setup MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect("localhost", 1883)

    while(""" !quit condition AKA Client is still running somehow """):
        key_man.iter()

        # TODO: Determine appropriate values for each command
        if key_man.w: publish_motors_vector(0, 0, 0, 0)
        elif key_man.a: publish_motors_vector(0, 0, 0, 0)
        elif key_man.s: publish_motors_vector(0, 0, 0, 0)
        elif key_man.d: publish_motors_vector(0, 0, 0, 0)

        client.loop()
    
    
if __name__ == "__main__":
	main()

