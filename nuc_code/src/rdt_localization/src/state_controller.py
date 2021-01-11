#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from rdt_localization.msg import *

# Digging zones
DIG_ZONE = Location()
DIG_ZONE.x = -2.5
DIG_ZONE.y = 2.7

# Topic names
TOPIC_FROM_HEARTBEAT_NODE = "server/heartbeat"
TOPIC_FROM_LOCALIZATION_NODE = "server/localization"
TOPIC_TO_PID_CONTROLLER_NODE = "server/orient_vector"
TOPIC_TO_DRIVE_NODE = "server/send_drive_vec"
TOPIC_TO_LIMB_NODE = "server/sendLimbVec"

ROSPY_LOOP_RATE = 10

# Most variables are open to being replaced
    # (I mean this in the sense that we can replace these variables with tangible conditions)
    # i.e. instead of if (robot_cannot_move), we might use something like if (motor_speed == 0)

# Robot variables
robot_state = 0
robot_pose = None
robot_localized = False
robot_cannot_move = False # If the robot cannot move
robot_exited_hole = False  # If the robot is out of the hole that it just dug
robot_face_depo = False # If the robot is facing the deposition zone
robot_in_depo_dist = False # If the robot is in distance to deposit
robot_depo_fail = False # If the robot crashes into the deposition bin or wall, or if the robot fails alignment
robot_unstable = False # If the robot becomes unstable while raising its frame
robot_face_dig_zone = False # If the robot is facing the digging zone

# Arm variables
arm_deployed = False # If the arms have been initially deployed
arm_stuck = False # If the arms are not being deployed despite being told to do so
arm_hit_surface = False  # If arms have contacted a surface
arm_min_extend = False # If the arms have reached their minimum extension
arm_gears_slip = False # If the arm gears have slipped
arm_rot_reached = False # If the preset arm rotation in preparation to continue moving
arm_drive_config = False # If the arm has extended to the driving configuration
arm_depo_ready = False # If the arms are in position to deposit

# Linear Actuator variables
lin_act_stuck = False # If the linear actuators are stuck
lin_act_ext = False # If the linear actuators are fully extended
lin_act_depo_ready = False # If the linear actuators are in position to deposit
lin_act_drive_config = False # If the linear actuators are in drive configuration

# Rest of the variables
e_stop = False # If the robot needs to emergency stop
inc_obstacle = False # If there are incoming obstacle
artag_seen = False # If April Tags can be located
drum_turning = False # If drums are turning
door_closed = False # If the door is closed
bin_full = False # If the storage bin is full
bin_empty = False # If the storage bin is empty

# Connection variables
drive_and_limbs_connected = False
sensor_connected = False
obstacle_connected = False

# Callback functions
def get_drive_and_limb_connection(data):
    global drive_and_limbs_connected
    drive_and_limbs_connected = True

def get_sensor_data(data):
    global sensor_connected
    sensor_connected = True

def get_obstable_data(data):
    global obstacle_connected
    obstacle_connected = True

def get_pose (data):
    global robot_localized
    global robot_pose
    robot_pose = data
    robot_localized = True

def main():
    global DIG_ZONE

    global robot_localized
    global robot_pose

    pid_pub = rospy.Publisher(TOPIC_TO_PID_CONTROLLER_NODE, Orientation_Vector, queue_size=10) # Output to PID controller node

    robot_state = 5
    #VERY TEMPORARY VARIABLE
    dig_zone_y_coord = 4.6

    # Setup ROS Node
    rospy.init_node('controller')
    rate = rospy.Rate(ROSPY_LOOP_RATE)

    # Timers
    max_manual_timer =  ROSPY_LOOP_RATE * 5 # If the robot doesn't fix itself in 5 seconds, it will be switched to manual
    manual_timer = max_manual_timer

    # Subscribers
    # Subscirbe to robot localization
    rospy.Subscriber(TOPIC_FROM_LOCALIZATION_NODE, Pose, get_pose)
    rospy.Subscriber("robotCmds/drive_and_limbs", String, get_drive_and_limb_connection)
    rospy.Subscriber("robotState/sensorData", String, get_sensor_data)
    rospy.Subscriber("robotState/obstacleData", String, get_obstable_data)

    # Handle input robot commands from GCS
    drive_string, limb_string = "", ""
    def parse_manual_commands (data):
        if (data[0] == "1"):
            drive_string = data.data[1:]
        elif (data[0] == "2"):
            limb_string = data.data[1:]

    rospy.Subscriber(TOPIC_FROM_HEARTBEAT_NODE, String, parse_manual_commands)



    # Publishers
    pub_drive_cmd = rospy.Publisher(TOPIC_TO_DRIVE_NODE, String, queue_size=10) # Drive commands
    pub_limb_cmd = rospy.Publisher(TOPIC_TO_LIMB_NODE, String, queue_size=10) # Limb commands



    # If the robot needs to emergency stop
    if (e_stop == True):
        # Switch to manual control
        pass

    while not rospy.is_shutdown():
        
        """
        # To see if the April Tags are seen
        if (-50 < robot_x < 50):
            artag_seen = True
            # If the robot is seen, refill the timer
            if (manual_timer < max_manual_timer):
                manual_timer += 1
        else:
            artag_seen = False
        """

        # if (manual_timer <= 0):
        #     # Switch to manual control
        #     pass



        # STATE 0: Manual state
        if robot_state == 0:
            # Relay commands from decoders to robot
            outmsg_drive = String()
            outmsg_drive.data = drive_string

            outmsg_limb = String()
            outmsg_limb = limb_string

            pub_drive_cmd.publish(outmsg_drive)
            pub_limb_cmd.publish(outmsg_limb)
            
        # STATE 1: Competition starts
        elif robot_state == 1:
            # Probably some setup code
            robot_state = 2

        # STATE 2: Machine connects to NUC
        elif robot_state == 2:
            if drive_connected and limbs_connected and sensor_connected and obstacle_connected:
                robot_state = 3

        # STATE 3: Initiate autonomy program
        elif robot_state == 3:
            # More setup code?
            robot_state = 4

        # STATE 4: Deploy Lifting Arms
        elif robot_state == 4:
            global state_4_start_time

            # Initialize state_4_start_time
            if(state_4_start_time == None): 
                state_4_start_time = time.time()
                rospy.loginfo("DEBUG: STATE 4")
            current_time = time.time() 

            # Try to localize robot for 30 seconds
            if robot_localized: 
                rospy.loginfo("DEBUG: ROBOT LOCALIZED")
                robot_state = 5

            # ERROR STATE: Ri4a
            elif current_time - state_4_start_time > 30:
                pass

        # STATE 5: Nuc localizes robot
        elif robot_state == 5:
            rospy.loginfo("DEBUG: STATE 5")
            # Make sure the April Tags are being detected
            if (robot_localized):
                robot_state = 6
            #else: # Error checking
                # Rotate robot and predetermine orientation of robot
                # Wait 5 seconds then switch to manual control
                #if (artag_seen == False):
                #    manual_timer -= 1

        # STATE 6: Machine moves to digging area
        elif robot_state == 6:
            rospy.loginfo("DEBUG: STATE 6")

            # Build Orientation_Vector consisting of bot and digging zone positions
            if not robot_pose == None:
                rospy.loginfo("DEBUG: PUBLISHING")

                outvec = Orientation_Vector()
                outvec.robot_pose = robot_pose
                outvec.dig_zone = DIG_ZONE
                outvec.robot_speed = 200

                pid_pub.publish(outvec)
            """
            # Error checking
            if (robot_cannot_move): #temp variable
                # find inverse of drive_string
                if drive_string[1] == 0:
                    inverse_drive_string =  str(0)+str(1)+drive_string[2:]
                else:
                    inverse_drive_string = str(0)+str(0)+drive_string[2:]
                # Find reverse of drive vector and input (backtrack)
                outmsg_drive.data = inverse_drive_string
                if robot_cannot_move:
                    # TIP(Turn in place) 
                    #setting outmsg_drive.data to the drivevector corresponding to TIP
                    #rotating 5 degrees positive
                    outmsg_drive.data = "1"+"000"+"005"
                    # If unsuccessful, switch to manual control
                pass
            elif (inc_obstacle):
                # TIP 
                # setting outmsg_drive.data to the drivevector corresponding to TIP
                # rotating 5 degrees positive
                while inc_obstacle:
                    outmsg_drive.data = "1"+"000"+"005"
                # Store obstacle loc as x & y coordinates for return 
                # If unsuccessful, switch to manual control
                manual_timer -= 1
                pass
            elif (artag_seen == False):
                # Store location of past, reverse drive vector
                if drive_string[1] == 0:
                    inverse_drive_string =  str(0)+str(1)+drive_string[2:]
                else:
                    inverse_drive_string = str(0)+str(0)+drive_string[2:]
                # Spin in place for 90 degs
                if artag_seen == False:
                    outmsg_drive.data = "1"+"000"+"090"
                # If unsuccessful, switch to manual control
                manual_timer -= 1
                pass

            if (robot_y > dig_zone_y_coord):
                robot_state = 7
            else:
                # Continue moving towards the digging area
                pass
            """

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
            else:
                # Continue lifting arms
                pass

        # STATE 14: Drum stops spinning
        elif robot_state == 14:
            if (drum_turning == False):
                robot_state = 15
            else:
                # Stop the drum from spinning
                pass

        # STATE 15: Orient to deposition zone
        elif robot_state == 15:
            # Error checking
            if (artag_seen == False):
                # This is different from the first resolution for this problem
                # Move robot slowly forward for 5 seconds
                # If unsuccessful, switch to manual control
                pass

            if (robot_face_depo):
                robot_state = 16
            else:
                # Continue orientating the robot
                pass

        # STATE 16: Machine navigates to some distance from depo zone
        elif robot_state == 16:
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

        # STATE 19: Linear actuators raise drum to upper limit
        elif robot_state == 19:
            if (lin_act_depo_ready):
                robot_state = 20
            else:
                # Continue raising the linear actuators
                pass

        # STATE 20: Open storage bin door to release payload
        elif robot_state == 20:
            if (bin_empty):
                robot_state = 21
            else:
                # Open the storage bin door
                pass

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
            if (artag_seen):
                robot_state = 25
            else:
                # Navigate back/diagonally until April Tags are sighted
                pass

        # STATE 25: Reorient robot to next digging zone
        else:
            if (robot_face_dig_zone):
                robot_state = 5
            else:
                # Continue reorientating robot
                pass

        rate.sleep()

        # In most of the states, the state will not simply be set to the next number
        # at the end of the 'elif.' Rather, it will be set to the next number once some
        # condition is met, such as 'digging zone reached' or 'drum full,' etc.



if __name__ == '__main__':
    main()
