#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from advanced_ros_demo.msg import Pose

def main():
    # Setup ROS Node
    rospy.Rate(50)
    rospy.init_node('controller')

    robot_state = 1

    # Publishing to the following topics:
    # pub1 = rospy.Publisher('robotData/TOPIC_NAME_1', MESSAGE_TYPE_1, queue_size=10)
    # pub2 = rospy.Publisher('robotData/TOPIC_NAME_2', MESSAGE_TYPE_2, queue_size=10)

    # Subscribing to the following topics:
    # rospy.Subscriber('TOPIC_NAME_3', MESSAGE_TYPE_3, CALLBACK_FUNCTION_3)
    # rospy.Subscriber('TOPIC_NAME_4', MESSAGE_TYPE_4, CALLBACK_FUNCTION_4)


    while True:

        # STATE 1: Competition starts
        if robot_state == 1:
            # Probably some setup code
            robot_state = 2

        # STATE 2: Machine connects to NUC
        elif robot_state == 2:
            # Code to connect the NUC to the robot using MQTT
            robot_state = 3

        # STATE 3: Intiate autonomy program
        elif robot_state == 3:
            # More setup code?
            robot_state = 4

        # And so on...
        # In most of the states, the state will not simply be set to the next number
        # at the end of the 'elif.' Rather, it will be set to the next number once some
        # condition is met, such as 'digging zone reached' or 'drum full,' etc.



if __name__ == '__main__':
    main()