import rospy 


def movementEncoder(input:list[bool]): 
    out = []
    out.append(0)
    if(input[0]): out[0]|=1
    if(input[1]): out[0]|=2
    if(input[2]): out[0]|=4
    if(input[3]): out[0]|=8
    return out 

def movementDecoder(data:list[char]):
    out = [] 
    if(data[0]&1): s+="W" 
    if(data[0]&2): s+="A" 
    if(data[0]&4): s+="S"
    if(data[0]&8): s+="D"
    rospy.loginfo(data) 
    


def run(argc:int, argv:str): 
    manager = Manager(argc, argv)
    manager.commands.insert(1, driveKey, driveEncoder, driveDecoder, driveSetup, driveCleanup);

    manager.commandsRos.AddBroadcast("slot1",None,None);
    manager.commandsRos.AddBroadcast("slot2",None,None);
    manager.commandsRos.AddBroadcast("slot3",None,None);
    manager.commandsRos.AddBroadcast("slot4",None,None);
    manager.commandsRos.AddBroadcast("slot5",None,None);
    manager.commandsRos.AddBroadcast("slot6",None,None);
    manager.commandsRos.AddBroadcast("slot7",None,None);
    manager.commandsRos.AddBroadcast("EMOJI", emojiSetup,emojiDecoder);
    manager.commandsRos.AddBroadcast("slot8",None,None);

    manager.loop()