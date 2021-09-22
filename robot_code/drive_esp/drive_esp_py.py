#!/usr/bin/env python3

import paho.mqtt.client as mqtt
from KeyManager import KeyManager

#TODO: Establish dummy vars for testing e.g. forward movement

def on_connect(client, userdata, flags, rc): 
    # print("Im connected yo")
    return 0 # delete later

# Convert motor values to bytes and publish it to the motors topic
def publish_motors_vector(motors):
    #recieve list of motors
    global client

    # Drive commands are sent through the MQTT network in the following format: 
    #TODO: Add the .msg format for motors
    #   motor1      motor2      motor3      motor4
    
    # encode all 4 fields (MOTORS) into 4 byte 
    rdt_data = bytes(motors)
    client.publish("robotCmds/motors", rdt_data) #TODO: Create new topic for motors - holds power levels of each motor

#kek there used to be 16 lines of code here but nobody will know what they were

def main():
    global client
    
    key_man = KeyManager()

    # Setup MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect("localhost", 1883)
    client.loop_start()
    while(not key_man.q):
	key_man.iter()

        # TODO: Determine appropriate values for each command
        motors = [0,0,0,0]
	update = False
	if key_man.w: 
		motors[0] = 1
        	update = True
	if key_man.a: 
		motors[1] = 1
        	update = True
	if key_man.s: 
		motors[2] = 1
        	update = True
	if key_man.d: 
		motors[3] = 1
		update = True

	if update: 
		publish_motors_vector(motors)
        # client.loop()

    client.loop_stop(force=False)
    
    
if __name__ == "__main__":
	main()

