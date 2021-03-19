from std_msgs.msg import String
from rdt_localization.msg import Drive_Vector

import paho.mqtt.client as mqtt
import rospy
import SampleAlgo

client = None

number_of_lidar = 2
LIDAR_DATA_TOPIC = "robotCmds/lidarData" #based on the non-working .ino file for lidar
STATE_CONTROLLER_OBSTACLE_TOPIC = "server/obstacle_data" #from the state controller
pub_lidar = rospy.Publisher(STATE_CONTROLLER_OBSTACLE_TOPIC, Obstacle, queue_size=10)

#Debug message if client connects
def on_connect():
    rospy.loginfo("DEBUG: ALGO BOUT TO GO WOOO")

#publish obstacle data to state controller from mqtt topic
def publish_obstacle_data(client,userdata,message):
    global algo
    o = Obstacle()
    if algo.calibration_ended:
        rospy.loginfo("Training wheels are off B)")
    else:
        rospy.loginfo("Training wheels still on =(")
    bool_array = algo.read(message)
    o.left = bool_array[0]
    o.right = bool_array[1]
    pub_lidar.publish(o)

def main():
    #Setup ROS Node
    
    algo = SampleAlgo(5)
    rospy.init_node("analyze_lidar_data")

    #Setup MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect("localhost",1883)
    client.on_message = publish_obstacle_data
    client.subscribe(LIDAR_DATA_TOPIC)

    while not rospy.is_shutdown():
        client.loop()

if __name__ == "__main__":
    main()
