"""
File used to receive data using MQTT client
Client is subscribed to topic "robotCmds/motors"
"""
import paho.mqtt.client as mqtt

# Callback function called each time data is received
def on_message(client, userdata, msg):
	print(".")
	print(type(msg.payload))
# Function called on client connection
def on_connect(client, userdata, flags, rc):	
	print("ProtoSub CONNECTED")

def main():
	global client
	
	# Instantiating client
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message
	#client.connect("192.168.1.10", 1883)
	
	#for testing, try connecting to localhost
	client.connect("localhost",1883)

	client.subscribe("robotCmds/motors")

	# TODO: add a real condition here
	# while(True) just for testing
	while(True):
		client.loop()


if __name__ == "__main__":
	main()
