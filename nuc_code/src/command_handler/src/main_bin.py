#!/usr/bin/env python

import rospy, sys
from GCSManager import GCSManager
from BINManager import BINManager
from KeyMap import *
from std_msgs.msg import String

from funcs import *

manual_drive_topic = "server/manual_drive"
manual_drive_sendHandler = rospy.Publisher(manual_drive_topic, String, queue_size=10)

keyboard_move_status = [False for i in range(4)] 

def movementEncoder(inp): 
    out = []
    out.append(0)
    if(inp[0]): out[0]|=1
    if(inp[1]): out[0]|=2
    if(inp[2]): out[0]|=4
    if(inp[3]): out[0]|=8
    return out 

def movementDecoder(data):
    s = ""
    if(data[0]&1): 
	s+="W" 
	keyboard_move_status[0] = True 
    else:
	keyboard_move_status[0] = False 
    if(data[0]&2): 
	s+="A" 
	keyboard_move_status[1] = True  
    else:
	keyboard_move_status[1] = False 
    if(data[0]&4): 
	s+="S"
	keyboard_move_status[2] = True 
    else:
	keyboard_move_status[2] = False 
    if(data[0]&8): 
	s+="D"
	keyboard_move_status[3] = True 
    else:
	keyboard_move_status[3] = False 
    print(keyboard_move_status)
    keyboard_data = ""
    keyboard_data += "T" if(keyboard_move_status[0]) else "F" 
    keyboard_data += "T" if(keyboard_move_status[1]) else "F"
    keyboard_data += "T" if(keyboard_move_status[2]) else "F"
    keyboard_data += "T" if(keyboard_move_status[3]) else "F"
    manual_drive_sendHandler.publish(keyboard_data)
    print(keyboard_data)
    rospy.loginfo(s) 

def testSetup(hi):
    pass

def testCleanup(hello):
    pass


def run(argc, argv): 
    #manager = GCSManager()
    rospy.init_node('BIN_command_handler')
    manager = BINManager()
    # manager.commands.insert(1, [Key_library.W, Key_library.A, Key_library.S, Key_library.D], encoder_manual_drive, decoder_manual_drive testSetup, testCleanup)
    
    # manager.commands.insert(1, [Key_library.M], encoder_emergency_stop, decoder_emergency_stop, testSetup, testCleanup)

    manager.commands.insert(1, [Key_library.W, Key_library.A, Key_library.S, Key_library.D], movementEncoder, movementDecoder, testSetup, testCleanup)
    
    manager.commands.insert(1, [Key_library.M], encoder_emergency_stop, decoder_emergency_stop, testSetup, testCleanup)

    manager.commands.insert(1, [Key_library.U, Key_library.J, Key_library.I, Key_library.K,Key_library.O, Key_library.L, Key_library.P ],encoder_limb, decoder_limb, None, None)

    manager.commandsRos.addBroadcast("slot1",None,None);
    manager.commandsRos.addBroadcast("slot2",None,None);
    manager.commandsRos.addBroadcast("slot3",None,None);
    manager.commandsRos.addBroadcast("slot4",None,None);
    manager.commandsRos.addBroadcast("slot5",None,None);
    manager.commandsRos.addBroadcast("slot6",None,None);
    manager.commandsRos.addBroadcast("slot7",None,None);
    #manager.commandsRos.addBroadcast("EMOJI", emojiSetup,emojiDecoder);
    manager.commandsRos.addBroadcast("slot8",None,None);

    manager.loop()

run(1, 1)


