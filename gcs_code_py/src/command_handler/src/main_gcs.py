#!/usr/bin/env python

import rospy, sys
from GCSManager import GCSManager
from BINManager import BINManager
from KeyMap import *
from funcs import *
#from command_handler.msg import *

def movementEncoder(inp): 
    out = []
    out.append(0)
    if(inp[0]): out[0]|=1
    if(inp[1]): out[0]|=2
    if(inp[2]): out[0]|=4
    if(inp[3]): out[0]|=8
    print(out)
    return out 

def movementDecoder(data):
    out = [] 
    if(data[0]&1): s+="W" 
    if(data[0]&2): s+="A" 
    if(data[0]&4): s+="S"
    if(data[0]&8): s+="D"
    rospy.loginfo(data) 

def testSetup(hi):
    pass

def testCleanup(hello):
    pass


def run(argc, argv): 
    rospy.init_node('GCS_command_handler')
    manager = GCSManager()
    #manager = BINManager()
    #manager.commands.insert(1, [Key_library.W, Key_library.A, Key_library.S, Key_library.D], movementEncoder, movementDecoder, testSetup, testCleanup);
    manager.commands.insert(1, [Key_library.W, Key_library.A, Key_library.S, Key_library.D], movementEncoder, movementDecoder, testSetup, testCleanup, "manual_drive", None)
    
    manager.commands.insert(1, [Key_library.M], encoder_emergency_stop, decoder_emergency_stop, testSetup, testCleanup, "manual_drive", None)

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


