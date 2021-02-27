#!/usr/bin/env python

"""
state_controller.py
The 'main' node in the ROS network - it interfaces with several other nodes to keep track of the
robot's state constantly. This includes the robot's location, the position of its limbs, and how
much load it is carrying.
Completed states: 2, 4, 5, 14, 15, 19, 20, 24, 27(Ri5B)
Tested states: 2, 4
"""

import rospy
import math

from std_msgs.msg import String
from std_msgs.msg import Bool
from rdt_localization.msg import *

# Whether or not to log status messages
RUN_VERBOSE = True

# Deposition zone
DEPOSITION_ZONE = Location()
DEPOSITION_ZONE.x = 0
DEPOSITION_ZONE.y = 0

# Digging zones
DIG_ZONE = Location()
DIG_ZONE.x = -2.5
DIG_ZONE.y = 2.7
ROSPY_LOOP_RATE = 20

# Robot constants
TARGET_TOLERANCE = 10                                   # Tolerance (cm) how far robot can stop from its target
ROTATION_TOLERANCE = 3                                     # Tolerance (deg) how close the robot should rotate to its target
TURN_IN_PLACE_SPEED = 50
MANUAL_SPEED_FWD = 200
MANUAL_SPEED_REVERSE = 0

# Topic names
TOPIC_FROM_HEARTBEAT_NODE = "server/heartbeat"          # Interface with heartbeat protocol to ensure connection
TOPIC_FROM_LOCALIZATION_NODE = "server/localization"    # Contains bot's pose/location
TOPIC_FROM_PING_NODE = "server/ping_drive_limb"         # Checks connection to drive/limb subsystems
TOPIC_FROM_SENSOR_NODE = "server/sensor_data"           # Contains the bot's last array of sensor data
TOPIC_FROM_OBSTACLE_NODE = "server/obstacle_data"       # Contains the bot's last array of obstacle data
TOPIC_TO_PID_CONTROLLER_NODE = "server/orient_vector"   # Passes pose/location to PID controller
TOPIC_TO_DRIVE_NODE = "server/send_drive_vec"           # Passes drive vector to send to bot
TOPIC_TO_LIMB_NODE = "server/send_limb_vec"             # Passes limb vector to send to bot
TOPIC_TO_MANUAL_DRIVE = "server/manual_drive"           # Receives key presses/releases from command handler

# Most variables are open to being replaced
# (I mean this in the sense that we can replace these variables with tangible conditions)
# i.e. instead of if (robot_cannot_move), we might use something like if (motor_speed == 0)

# Robot variables
robot_state = 4
robot_pose = None
robot_localized = False
robot_cannot_move = False             # If the robot cannot move
robot_exited_hole = False             # If the robot is out of the hole that it just dug
robot_face_depo = False               # If the robot is facing the deposition zone
robot_in_depo_dist = False            # If the robot is in distance to deposit
robot_depo_fail = False               # If the robot crashes into the deposition bin or wall, or if the robot fails alignment
robot_unstable = False                # If the robot becomes unstable while raising its frame
robot_face_dig_zone = False           # If the robot is facing the digging zone
    
# Arm variables       
arm_deployed = False                  # If the arms have been initially deployed
arm_stuck = False                     # If the arms are not being deployed despite being told to do so
arm_hit_surface = False               # If arms have contacted a surface
arm_min_extend = False                # If the arms have reached their minimum extension
arm_gears_slip = False                # If the arm gears have slipped
arm_rot_reached = False               # If the preset arm rotation in preparation to continue moving
arm_drive_config = False              # If the arm has extended to the driving configuration
arm_depo_ready = False                # If the arms are in position to deposit

# Linear Actuator variables       
lin_act_stuck = False                 # If the linear actuators are stuck
lin_act_ext = False                   # If the linear actuators are fully extended
lin_act_depo_ready = False            # If the linear actuators are in position to deposit
lin_act_drive_config = False          # If the linear actuators are in drive configuration

# Rest of the variables   
e_stop = False                        # If the robot needs to emergency stop
inc_obstacle = None                  # If there are incoming obstacle
artag_seen = False                    # If April Tags can be located
drum_turning = False                  # If drums are turning
door_closed = False                   # If the door is closed
bin_full = False                      # If the storage bin is full
bin_empty = False                     # If the storage bin is empty

# Comms variables
last_keys = None                      # Last received set of manual commands
drive_and_limbs_connected = False
sensor_connected = False
obstacle_connected = False


# ROS callback functions
'''
Gets the last set of key presses/releases from the GCS.
data: rdt_localization/Keyboard
'''
def manualDriveRecieve(data): 
    global last_keys
    previous = last_keys
    last_keys = data 

'''
Gets response from ping_drive_limb node to ensure that the locomotion and limbs subsystems are 
active and connected.
data: std_msgs/Bool
'''
def get_drive_and_limb_connection(data):
    global drive_and_limbs_connected
    drive_and_limbs_connected = True

'''
Gets last array of sensor data from the read_sensor_data node.
data: TBD
'''
def get_sensor_data(data):
    global sensor_connected
    sensor_connected = True

'''
Gets the last array of obstacle data from the read_obstacle_data node.
data: TBD
'''
def get_obstacle_data(data):
    global obstacle_connected
    global inc_obstacle
    inc_obstacle = data
    obstacle_connected = True

'''
Gets the last known position and orientation of the robot from the aptag_optimized node.
data: rdt_localization/Pose
'''
def get_pose(data):
    global robot_localized
    global robot_pose
    robot_pose = data
    robot_localized = True

'''
Reads a manual command sent from the GCS and processes it.
data: TBD
'''
def parse_manual_commands(data):
    pass


# Helper functions
def ros_log(info):
    global RUN_VERBOSE
    if RUN_VERBOSE:
        rospy.loginfo(info)

'''
Parses the last received keyboard command and composes output vectors accordingly.
drive_vec: rdt_localization/Drive_Vector
'''
def process_manual_cmd(drive_vec):
    global last_keys

    # WA command
    if (last_keys.w and last_keys.a):
        drive_vec.offset_driveMode = 50
        drive_vec.robot_spd = MANUAL_SPEED_FWD

    # WD command
    elif (last_keys.w and last_keys.d):
        drive_vec.offset_driveMode = 150
        drive_vec.robot_spd = MANUAL_SPEED_FWD

    # SA command
    elif (last_keys.s and last_keys.a):
        drive_vec.offset_driveMode = 50
        drive_vec.robot_spd = MANUAL_SPEED_REVERSE
    
    # SD command
    elif (last_keys.s and last_keys.d):
        drive_vec.offset_driveMode = 150
        drive_vec.robot_spd = MANUAL_SPEED_REVERSE
        
    # W command  
    elif(last_keys.w):
        drive_vec.offset_driveMode = 100
        drive_vec.robot_spd = MANUAL_SPEED_FWD

    # A command
    elif(last_keys.a):
        drive_vec.offset_driveMode = 254
        drive_vec.robot_spd = 0

    # S command
    elif(last_keys.s): 
        drive_vec.offset_driveMode = 100
        drive_vec.robot_spd = MANUAL_SPEED_REVERSE

    # D command
    elif(last_keys.d):
        drive_vec.offset_driveMode = 254 
        drive_vec.robot_spd = 200

    # no keys are pressed
    else:
        drive_vec.offset_driveMode = 100
        drive_vec.robot_spd = 100

    return drive_vec

def main():
    global robot_state
    global DIG_ZONE
    global robot_localized
    global robot_pose

    # Setup ROS Node
    rospy.init_node('controller')
    rate = rospy.Rate(ROSPY_LOOP_RATE)

    # Subscribers
    rospy.Subscriber(TOPIC_FROM_LOCALIZATION_NODE, Pose, get_pose)
    rospy.Subscriber(TOPIC_FROM_PING_NODE, Bool, get_drive_and_limb_connection)
    rospy.Subscriber(TOPIC_FROM_SENSOR_NODE, String, get_sensor_data)
    rospy.Subscriber(TOPIC_FROM_OBSTACLE_NODE, Obstacle, get_obstacle_data)
    rospy.Subscriber(TOPIC_FROM_HEARTBEAT_NODE, String, parse_manual_commands)
    rospy.Subscriber(TOPIC_TO_MANUAL_DRIVE, String, manualDriveRecieve)

    # Publishers
    pub_pid = rospy.Publisher(TOPIC_TO_PID_CONTROLLER_NODE, Orientation_Vector, queue_size=10)
    pub_drive_cmd = rospy.Publisher(TOPIC_TO_DRIVE_NODE, Drive_Vector, queue_size=10)
    pub_limb_cmd = rospy.Publisher(TOPIC_TO_LIMB_NODE, Limb_Vector, queue_size=10)

    # Timers
    curr_state_start_time = None

    # Main loop
    while not rospy.is_shutdown():
        # STATE 0: Manual state
        if robot_state == 0:
            global last_keys 

            # Process the last received manual commands and send them
            drive_vec = Drive_Vector() 
            process_manual_cmd(drive_vec)
            pub_drive_cmd.publish(drive_vec)
            
        # STATE 1: Competition starts
        elif robot_state == 1:
            # Probably some setup code
            robot_state = 2

        # STATE 2: Machine connects to NUC
        elif robot_state == 2:
            ros_log("DEBUG: STATE 2")
            # if drive_and_limbs_connected and sensor_connected and obstacle_connected:
            if drive_and_limbs_connected:
                robot_state = 3

        # STATE 3: Initiate autonomy program
        elif robot_state == 3:
            ros_log("DEBUG: STATE 3")
            # More setup code?
            robot_state = 4

        # STATE 4: Deploy Lifting Arms
        elif robot_state == 4:
            # Initialize state_4_start_time
            if(curr_state_start_time == None): 
                curr_state_start_time = rospy.get_time()
                ros_log("DEBUG: STATE 4")
            current_time = rospy.get_time() 

            # Try to localize robot for 30 seconds
            if robot_localized: 
                ros_log("DEBUG: ROBOT LOCALIZED")
                curr_state_start_time = None
                robot_state = 5

            # ERROR STATE: Ri4a
            elif current_time - curr_state_start_time > 30:
                pass

        # STATE 5: Nuc localizes robot
        elif robot_state == 5:
            ros_log("DEBUG: STATE 5")
            if inc_obstacle is not None and (inc_obstacle.left or inc_obstacle.right):
                robot_state = 27

            # Build Orientation_Vector consisting of bot and digging zone positions
            elif not robot_pose == None:
                ros_log("DEBUG: PUBLISHING")

                outvec = Orientation_Vector()
                outvec.robot_pose = robot_pose
                outvec.target_zone = DIG_ZONE
                outvec.robot_speed = 200

                pub_pid.publish(outvec)

        # STATE 6: Machine moves to digging area
        elif robot_state == 6:
            x_difference = abs(DIG_ZONE.x - robot_pose.x)
            y_difference = abs(DIG_ZONE.y - robot_pose.y)

            if x_difference < TARGET_TOLERANCE and y_difference < TARGET_TOLERANCE: #digging zone reached
                robot_state = 7

            if not robot_pose == None:
                ros_log("DEBUG: PUBLISHING STATE 6")
                
                outvec = Orientation_Vector()
                outvec.robot_pose = robot_pose
                outvec.target_zone = DIG_ZONE
                outvec.robot_speed = 200

                pub_pid.publish(outvec)

        # STATE 7: Drum begins to turn
        elif robot_state == 7:
            if (drum_turning):
                robot_state = 8
            else:
                # Start turning drums
                pass

        # STATE 8: Arms lower drum until contact
        elif robot_state == 8:
            # Error checking
            if (arm_gears_slip):
                # Rotate in opposite direction and keep encoder rotation at 0
                pass

            if (arm_hit_surface or arm_min_extend):
                robot_state = 9
            else:
                # Continue lowering arms
                pass

        # STATE 8: Linear actuators push drum down to dig
        elif robot_state == 9:
            # Error checking
            if (lin_act_stuck):
                # Shift around until there is movement
                pass

            if (bin_full or lin_act_ext):
                robot_state = 10
            else:
                # Continue pushing drum down to dig
                pass

        # STATE 9: Linear actuators lift drum
        elif robot_state == 10:
            if (bin_full or lin_act_ext):
                robot_state = 11
            else:
                robot_state = 8

        # STATE 11: Arms lift drum until it is just below surface level
        elif robot_state == 11:
            if (arm_rot_reached):
                robot_state = 12
            else:
                # Continue lifting drum
                pass

        # STATE 12: Move forward until all wheels are in front of the hole
        elif robot_state == 12:
            # Error checking
            if (inc_obstacle):
                # Move around the obstacle using sensors
                # If unsuccessful, switch to manual control
                pass

            if (robot_exited_hole):
                robot_state = 13
            else:
                # Continue moving over the hole
                pass

        # STATE 13: Lift arms into driving configuration
        elif robot_state == 13:
            if (arm_drive_config):
                robot_state = 14
            else: #continue digging
                arm_vec = Limb_Vector()
                arm_vec.linActs_speed = 0
                arm_vec.arm_speed = 0
                arm_vec.drum_speed = 0

                arm_vec.door = False 

                pub_limb_cmd.publish(arm_vec) 

        # STATE 14: Drum stops spinning
        elif robot_state == 14:
            # Error checking
            if (artag_seen == False):
                # This is different from the first resolution for this problem
                # Move robot slowly forward for 5 seconds
                # If unsuccessful, switch to manual control
                pass
            ros_log("desired orient:" + str(((math.atan((robot_pose.x) / robot_pose.y)) * 180 / math.pi)+180))
            ros_log("orientation:" + str(robot_pose.orientation))
            ros_log("x:" + str(robot_pose.x))
            ros_log("y:" + str(robot_pose.y))
            if ROTATION_TOLERANCE >= abs(((math.atan((robot_pose.x) / robot_pose.y)) * 180 / math.pi)+180 - robot_pose.orientation):
                ros_log("robot correct")
                robot_state = 15
            else:
                # Robot rotates in place
                outmsg = Drive_Vector()
                outmsg.offset_driveMode = 254
                outmsg.robot_spd = 150

                pub_drive_cmd.publish(outmsg)

        # STATE 15: Navigating from digging zone to deposition zone
        elif robot_state == 15:
            # Continue navigating to correct distance from deposition zone
            x_difference = abs(DEPOSITION_ZONE.x - robot_pose.x)
            y_difference = abs(DEPOSITION_ZONE.y - robot_pose.y)

            # Check that robot is within tolerance at depo zone
            if (x_difference < TARGET_TOLERANCE) and (y_difference < TARGET_TOLERANCE):
                robot_state = 16
            if not robot_pose == None:
                ros_log("DEBUG: PUBLISHING")
                
                outvec = Orientation_Vector()
                outvec.robot_pose = robot_pose
                outvec.target_zone = DEPOSITION_ZONE
                outvec.robot_speed = 200

                pub_pid.publish(outvec)      

        # STATE 16: Machine navigates to some distance from depo zone
        elif robot_state == 16:
            ros_log("DEBUG: STATE 16")
            # Error checking
            if (robot_cannot_move):  # temp variable
                # Find reverse of drive vector and input (backtrack)
                # If unsuccessful, switch to manual control
                pass
            elif (inc_obstacle):
                # Move around the obstacle using sensors
                # If unsuccessful, switch to manual control
                pass
            elif (artag_seen == False):
                # Spin in place (3 rotations per 10 seconds)
                # If unsuccessful, switch to manual control
                pass

            if (robot_face_depo and robot_in_depo_dist):
                robot_state = 17
            else:
                # Continue navigating to correct distance from deposition zone
                pass

        # STATE 17: Orient with deposition bin
        elif robot_state == 17:
            # Error checking
            if (robot_depo_fail):
                # Switch to manual control
                pass

            if (robot_face_depo):
                robot_state = 18
            else:
                # Continue orientating the robot
                pass

        # STATE 18: Arms raise frame to upper limit
        elif robot_state == 18:
            if (robot_unstable):
                # Extend arms slightly to distribute weight
                pass

            if (arm_depo_ready):
                robot_state = 19
            else:
                # Continue raising the arms
                pass

        # STATE 19: Open storage bin door to release payload
        elif robot_state == 19:
            if (bin_empty):
                robot_state = 20
            else:
                # Send out 'open door' vector
                outvec = Limb_Vector()
                outvec.linActs_speed = 0
                outvec.arm_speed = 0
                outvec.drum_speed = 0

                outvec.door = True 

                pub_limb_cmd.publish(outvec)

        # STATE 20: Close door
        elif robot_state == 20:
            if (door_closed):
                robot_state = 21
            else:
                # Send out 'close door' vector
                outvec = Limb_Vector()
                outvec.linActs_speed = 0
                outvec.arm_speed = 0
                outvec.drum_speed = 0

                outvec.door = False 

                pub_limb_cmd.publish(outvec)

        # STATE 21: Close door
        elif robot_state == 21:
            if (door_closed):
                robot_state = 22
            else:
                # Close the door
                pass

        # STATE 22: Lower linear actuators into driving configuration
        elif robot_state == 22:
            if (lin_act_drive_config):
                robot_state = 23
            else:
                # Continue lowering linear actuators
                pass

        # STATE 23: Lower arms into driving configuration
        elif robot_state == 23:
            if (arm_drive_config):
                robot_state = 24
            else:
                # Continue lowering arms
                pass

        # STATE 24: Navigate back/diagonally until April Tags sighted
        elif robot_state == 24:
            # Error checking
            if not artag_seen:
                # (Keep rotating)
                pass

            # Use law of cosines to find desired angle (c^2 = a^2 + b^2 - 2ab * cos(C))
            # Where a, b, c = triangle sides and C = desired angle
            # Three points will be robot robot_x/y++, x/y, and the dig zone
            top_p = (robot_pose.x, robot_pose.y + 1)
            robot_p = (robot_pose.x, robot_pose.y)
            dig_p = (DIG_ZONE.x, DIG_ZONE.y)

            # I am using distance formula (dist = sqrt((x2 - x1)^2 + (y2 - y1)^2))
            a_sq = ((top_p[0] - robot_p[0]) ** 2 ) + ((top_p[1] - robot_p[1]) ** 2)
            b_sq = ((dig_p[0] - robot_p[0]) ** 2 ) + ((dig_p[1] - robot_p[1]) ** 2)
            c_sq = ((top_p[0] - dig_p[0]) ** 2 ) + ((top_p[1] - dig_p[1]) ** 2)

            a = math.sqrt(a_sq)
            b = math.sqrt(b_sq)
            
            desired_orient = math.acos((a_sq + b_sq - c_sq) / (2 * a * b))
            
            ros_log("desired orient:" + desired_orient)
            ros_log("orientation:" + str(robot_pose.orientation))
            ros_log("x:" + str(robot_pose.x))
            ros_log("y:" + str(robot_pose.y))
            
            if ROTATION_TOLERANCE >= abs(desired_orient - robot_pose.orientation):
                ros_log("robot correct")
                robot_state = 4  # The next state
            else:
                # Robot rotates in place
                outmsg = Drive_Vector()
                outmsg.offset_driveMode = 254  # Turn in place
                outmsg.robot_spd = 150  # 50% speed forward rotation
            
                pub_drive_cmd.publish(outmsg)

        # STATE Ri5B: Error state of obstacle detected
        elif robot_state == 27:
            ros.log("DEBUG STATE 27")
            # If no more obstacle detected, transition back to state 5
            if (not (inc_obstacle.left or inc_obstacle.right)):
                robot_state = 5
            else:
                # Turn left if obstacle to the right, & vice versa
                if (inc_obstacle.left):
                    speed = TURN_IN_PLACE_SPEED
                else:
                    speed = 200 - TURN_IN_PLACE_SPEED
                    
                # Send out correcting drive vector to turn bot in place
                rot = Drive_Vector()
                rot.offset_driveMode = 254
                rot.robot_spd = speed
                pub_drive_cmd.publish(rot)

        rate.sleep()


if __name__ == '__main__':
    main()
