#!/usr/bin/env python
import rospy
import time
import math

from std_msgs.msg import String
from rdt_localization.msg import Pose
from rdt_localization.msg import Drive_Vector
from rdt_localization.msg import Orientation_Vector

from pid_controller import PID
#This line was commented out bcuz mttkinter cannot be install on nuc.
#from pid_simulator import PID_simulator
G_KP = 0.5
G_KI = 0.0
G_KD = 0.0
G_WINDUP = 360.0
pub = rospy.Publisher('server/send_drive_vec', Drive_Vector, queue_size=10)
controller = PID(G_KP, G_KI, G_KD)
simulator = PID_simulator()
"""
Callback function executed every time a new Orientation_Vector is received on orient_vector
"""
simulator = PID_simulator(controller)
def run_pid(data):
    global G_KP,G_KD,G_WINDUP
    global simulator
    global pub
    angle_error = PID_error(data)

    #offset = simulator.update_feedback(angle_error)
    #The above line was commented out because mttkinter from pid_simulator cannot be pip install on the nuc.
    #Once it is, we can try to restore this line to get a mt
    offset = controller.update(angle_error)

    simulator.update_feedback(angle_error)

    rospy.loginfo(offset)
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
    displacement = (float(data.dig_zone.x) - float(data.robot_pose.x), float(data.dig_zone.y) - float(data.robot_pose.y))

    # Angular offset between camera forward line and line between digging zone/bot
    desired_theta = math.degrees(math.atan2(displacement[0], displacement[1]))
    
    # Range of angle_error: [-180, 180]
    angle_error = abs(desired_theta - robot_orient)
    '''
    if desired_theta < 0 and robot_orient > 0:
        angle_error = 360 - (abs(angle_error))
    elif desired_theta > 0 and robot_orient < 0:
        angle_error = -(360-abs(angle_error))
    '''

    # dont rely on this idk what im doing
    turn_left = desired_theta > robot_orient

    #angle error > 180, rather turn the other way bcuz speed
    if angle_error > 180:
        turn_left =  not turn_left
        angle_error = 360 - angle_error%360 #should cover when angle_error > 360 if bug happens

    if not turn_left: #tba for flippy flip
        angle_error *= -1 
    
    return angle_error

def main():
    rospy.init_node('orientation_pid_node')
    rospy.Subscriber("server/orient_vector", Orientation_Vector, run_pid)
    rospy.spin()
    
if __name__ == '__main__':
    main()