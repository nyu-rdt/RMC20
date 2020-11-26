#!/usr/bin/env python

# TODO:
# matplotlib tool for fine-tuning (last-priority)

import rospy
import time
import math

from std_msgs.msg import String
from rdt_localization.msg import Pose
from rdt_localization.msg import Drive_Vector
from rdt_localization.msg import Orientation_Vector

from pid_controller import PID

G_KP = 0.0
G_KI = 0.0
G_KD = 0.0
G_WINDUP = 360.0
pub = rospy.Publisher('robotData/drive_vector', Drive_Vector, queue_size=10)

"""
Callback function executed every time a new Orientation_Vector is received on orient_vector
"""
def run_pid(data):
    controller = PID(G_KP, G_KI, G_KD)
    angle_error = PID_error(data)
    offset = controller.update(angle_error)
    
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
    desired_theta = math.degrees(math.atan2(displacement[0]/displacement[1]))
    
    '''
    angle_error = desired_theta - robot_orient
    Desired theta & orientation can still be negative if >180. Possible edge cases:
    Digging angle is >180 => desired_theta is negative:
        robot_orient is <180 => robot_orient positive
        we want to turn +(360-|angle_error|)
    Digging angle is <180 => desired_theta is positive
        robot_orient is >180 => robot_orient negative
        we want to turn -(360-|angle_error|)
        wait this is wrong ffffffffffffffff
    '''
    #OOF
    #plz do vector my brain can't handle edge cases thx
    # Range of angle_error: [-180, 180]
    angle_error = (desired_theta - robot_orient)
    if desired_theta < 0 and robot_orient > 0:
        angle_error = 360 - (abs(angle_error))
    elif desired_theta > 0 and robot_orient < 0:
        angle_error = -(360-abs(angle_error))
    return angle_error

def main():
    rospy.Subscriber("robotData/orient_vector", Orientation_Vector, run_pid)
    rospy.spin()
    
if __name__ == '__main__':
    main()