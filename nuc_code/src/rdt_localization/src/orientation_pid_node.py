#!/usr/bin/env python

"""
orientation_pid_node.py

Node that subscribes to state_controller and utilizes the given info to create an appropriate 
outmsg to the send_drive_vector node using a PID controller. Used to ensure the robot takes an 
efficient path to its destination .
"""

import rospy
import time
import math

from std_msgs.msg import String
from rdt_localization.msg import Pose
from rdt_localization.msg import Drive_Vector
from rdt_localization.msg import Orientation_Vector

from pid_controller import PID

# NOTE, anything involving "pid_simulator" has been commented out because it is not 100% functional yet
# If the simulator never ends up working, we can just remove those lines of code

#from pid_simulator import PID_simulator

# TODO: Tune P, I, D constants
#Global Variables
G_KP = 10
G_KI = 0.0
G_KD = 0.0
G_WINDUP = 360.0
pub = rospy.Publisher('server/send_drive_vec', Drive_Vector, queue_size=10)
controller = PID(G_KP, G_KI, G_KD)
# simulator = PID_simulator()
# simulator = PID_simulator(controller)

"""
Callback function executed every time a new Orientation_Vector is received on orient_vector
"""
def run_pid(data):
    global G_KP,G_KD,G_WINDUP
    # global simulator
    global pub
    angle_error = PID_error(data)

    # offset = simulator.update_feedback(angle_error) Commented out because mttkinter from pid_simulator cannot be pip install on the nuc

    offset = controller.update(angle_error)

    # simulator.update_feedback(angle_error)

    rospy.loginfo(angle_error)

    # PID outputs in the following range:
    # [(G_KP*-180)-(G_KD*-360)-G_WINDUP, (G_KP*180)+(G_KD*360)+G_WINDUP]
    # We want to scale the output to [0, 200] to be compliant with the offset format
    old_min = (G_KP*-180)-(G_KD*-360)-G_WINDUP
    old_max = (G_KP*180)+(G_KD*360)+G_WINDUP
    n_offset = ((offset-old_min)/(old_max-old_min))*(200)
    
    # Generate and publish new drive vector
    outmsg = Drive_Vector()
    outmsg.offset_driveMode = n_offset
    outmsg.robot_spd = data.robot_speed

    pub.publish(outmsg)

"""
Calculate the current robot error based on its current pose and the digging zone location
"""
def PID_error(data):
    # Get locations/orientations of robot and dig zone
    robot_orient = float(data.robot_pose.orientation)

    # Displacement between the center of the robot and the digging zone
    displacement = (float(data.target_zone.x) - float(data.robot_pose.x), float(data.target_zone.y) - float(data.robot_pose.y))

    # Angular offset between camera forward line and line between digging zone/bot
    desired_theta = math.degrees(math.atan2(displacement[0], displacement[1]))
    
    # Initial range of angle_error: [0, 360]
    angle_error = abs(desired_theta - robot_orient)%360
    # TODO: Determine if the directions need to be negated 
    # Determining whether we want to turn left
    turn_left = desired_theta > robot_orient

    # If the error is more than 180, turn the opposite direction
    if angle_error > 180:
        turn_left = not turn_left
        angle_error = 360 - angle_error%360 

    # If we're not turning left, error should be negative
    if not turn_left:
        angle_error *= -1 
    
    # Final range of angle_error: [-180, 180]
    return angle_error

def main():
    rospy.init_node('orientation_pid_node')
    rospy.Subscriber("server/orient_vector", Orientation_Vector, run_pid)
    rospy.spin()
    
if __name__ == '__main__':
    main()
