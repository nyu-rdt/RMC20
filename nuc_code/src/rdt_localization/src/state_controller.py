#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from rdt_localization.msg import Pose

# Topic names
TOPIC_FROM_HEARTBEAT_NODE = "server/heartbeat"
TOPIC_FROM_LOCALIZATION_NODE = "server/localization"
TOPIC_TO_DRIVE_NODE = "server/sendDriveVec"
TOPIC_TO_LIMB_NODE = "server/sendLimbVec"

ROSPY_LOOP_RATE = 50

# Most variables are open to being replaced
    # (I mean this in the sense that we can replace these variables with tangible conditions)
    # i.e. instead of if (robot_cannot_move), we might use something like if (motor_speed == 0)

# Robot variables
robot_state = 0
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

def main():
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
    robot_x, robot_y, robot_orientation = None, None, None
    def get_pose (data):
        robot_x = data.x
        robot_y = data.y
        robot_orientation = data.orientation
    rospy.Subscriber(TOPIC_FROM_LOCALIZATION_NODE, Pose, get_pose)

    # Handle input robot commands from GCS
    drive_string, limb_string = "", ""
    def parse_manual_commands (data):
        if (data[0] == "1"):
            drive_string = data.data[1:]
        elif (data[0] == "2"):
            limb_string = data.data[1:]

    rospy.Subscriber(HEARTBEAT_NODE, String, parse_manual_commands)



    # Publishers
    pub_drive_cmd = rospy.Publisher('robotCmds/Drive', Pose, queue_size=10) # Drive commands
    pub_limb_cmd = rospy.Publisher('robotCmds/Limbs', Pose, queue_size=10) # Limb commands



    # If the robot needs to emergency stop
    if (e_stop == True):
        # Switch to manual control
        pass

    while True:
        # To see if the April Tags are seen
        if (-50 < robot_x < 50):
            artag_seen = True
            # If the robot is seen, refill the timer
            if (manual_timer < max_manual_timer):
                manual_timer += 1
        else:
            artag_seen = False

        if (manual_timer <= 0):
            # Switch to manual control
            pass



        # STATE 0: Manual state
        if robot_state == 0:
            # Relay commands from decoders to robot
            pub_drive_cmd.publish(drive_string)
            pub_limb_cmd.publish(limb_string)

        # STATE 1: Competition starts
        elif robot_state == 1:
            # Probably some setup code
            robot_state = 2

        # STATE 2: Machine connects to NUC
        elif robot_state == 2:
            # Code to connect the NUC to the robot using MQTT
            robot_state = 3

        # STATE 3: Initiate autonomy program
        elif robot_state == 3:
            # More setup code?
            robot_state = 4

        # STATE 4: Deploy Lifting Arms
        elif robot_state == 4:
            # Error checking
            if (arm_stuck):
                manual_timer -= 1
            else:
                manual_timer = max_manual_timer

            if (arm_deployed == False):
                # Continue deploying arms
                pass
            else:
                robot_state = 5

        # STATE 5: Nuc localizes robot
        elif robot_state == 5:
            # Make sure the April Tags are being detected
            if (robot_localized):
                robot_state = 6
            else: # Error checking
                # Rotate robot and predetermine orientation of robot
                # Wait 5 seconds then switch to manual control
                if (artag_seen == False):
                    manual_timer -= 1;

        # STATE 5: Machine moves to digging area
        elif robot_state == 6:
            # Error checking
            if (robot_cannot_move): #temp variable
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
                manual_timer -= 1;
                pass

            if (robot_y > dig_zone_y_coord):
                robot_state = 7
            else:
                # Continue moving towards the digging area
                pass

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

        # In most of the states, the state will not simply be set to the next number
        # at the end of the 'elif.' Rather, it will be set to the next number once some
        # condition is met, such as 'digging zone reached' or 'drum full,' etc.



if __name__ == '__main__':
    main()