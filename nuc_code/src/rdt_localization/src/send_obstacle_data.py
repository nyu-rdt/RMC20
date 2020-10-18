
#!/usr/bin/env python

from std_msgs.msg import String

import paho.mqtt.client as mqtt
import rospy
import math

# callback function gets executed when connection is made to server
def on_connect(client, userdata, flags, rc):
	rospy.loginfo("Connected: receiving obstacle data")

# callback function gets executed when topic "robotCmds/sendObstacleData" receives new drive_vector
def publish_obstacle_data(data):
	obstacle = str(data.data)
	client.publish("server/receiveObstacleData", indicatingObstacles(obstacle))
#take in 4 floats of the distance observed by each lidar
#return string indicates whether there is an obstacle
#input format: 
#11\n 10\n 30\n 35\n
#output format: 
#left true\n right false
#optional input: vertical distance, theta of sensors (in radian), margin of error 

#note: does not account for curvature
def indicatingObstacles(input, ydist = 4.5, theta = math.pi/4, margin = 0.2):
	name = ["outer left","inner left", "inner right","outer right"]
	lst = input.splitlines()
	ret = []
	acceptabledist = ydist/math.cos(theta) if theta<math.pi/2 else 0
	for i in range(len(lst)):
		try:
			dist = float(lst[i])
			if acceptabledist*(1-margin)<=dist<=acceptabledist*(1+margin):
				ret.append("false")
			else:
				ret.append("true")
		except:
			pass
	return "\n".join(ret)

def state_controller_listener():
	rospy.init_node("send_obstacle_data")
	rospy.Subscriber("robotCmds/obstacle", String, publish_obstacle_data)
	
	client = mqtt.Client()
	client.on_connect = on_connect		
	
	# connects to the server on the NUC
	client.connect("localhost", 1883)

	# THIS LINE IS FOR TESTING, DELETE LATER
	#for i in range(0, 10):		
	#	client.publish("robotCmds/drive", "00100000")

	#rospy.spin()
	while not rospy.is_shutdown():
		client.loop()

if __name__ == "__main__":
	state_controller_listener()
