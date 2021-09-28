#!/usr/bin/env python3

import paho.mqtt.client as mqtt
from KeyManager import KeyManager

#TODO: Establish dummy vars for testing e.g. forward movement

def on_connect(client, userdata, flags, rc): 
    return 0

# Convert motor values to bytes and publish it to the motors topic
def publish_motors_vector(motors):
    global client

    # Drive commands are sent through the MQTT network in the following format: 
    # 1 byte, 4 bits for each side (Left side, right side)
    # First bit determines direction, last 3 bits determine power

    # Encoding data as 1 byte
    # prolly will combine parameter into 1 element list im just lazy rn
    motor_byte = (motors[0] << 4) | motors[1]
    # motor_byte = 170
    rdt_data = bytes([motor_byte])
    client.publish("robotCmds/motors", rdt_data)

def main():
    global client

    key_man = KeyManager()

    # Setup MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect("localhost", 1883)
    client.loop_start()

    prev_w = 0;
    prev_s = 0;
    prev_a = 0;
    prev_d = 0;

    while(not key_man.q):
	key_man.iter()

	# TODO: Determine appropriate values for each command
	# TODO: Fix issue with update not registering key releases
	motors = [0,0]
	# can definitely be cleaned but someone else can do it
	update = key_man.w ^ prev_w or key_man.a ^ prev_a or key_man.s ^ prev_s or key_man.d ^ prev_d
	if update:
		if key_man.w: 
			motors[0] = 10 # 4'b1010
			motors[1] = 10 # 4'b1010
		elif key_man.a: 
			motors[0] = 2 # 4'b0010
			motors[1] = 10 # 4'b1010
		elif key_man.s: 
			motors[0] = 2 # 4'b0010
			motors[1] = 2 # 4'b0010
		elif key_man.d: 
			motors[0] = 10 # 4'b1010
			motors[1] = 2 # 4'b0010
		if update:
			publish_motors_vector(motors)
	prev_w = key_man.w;
	prev_s = key_man.s;
	prev_a = key_man.a;
	prev_d = key_man.d;
	#Testing push 2

    client.loop_stop(force=False)


if __name__ == "__main__":
	main()
