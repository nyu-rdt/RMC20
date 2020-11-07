# convert Manager to python
# https://github.com/nyu-rdt/RMC20/blob/ground_control_station/gcs_code/src/framework/src/Manager.cpp


#!/usr/bin/env python
# import filenames-interpreter and user-input libraries
import sys, pygame, rospy, KeyMap

from std_msgs.msg import String


# pop up status indicator window for connection status 
# green circle for good connection, red for bad
class BINManager:
    def __init__(self, title="  RDT Command Framework":str, width=640:int, height=480:int):
        
        self.commands = FunctionTable() 
        self.commandsRos = RosTable() 
        
        # ros init with command line arguments, needs to be finished. possible line of code:
        # rospy.init_node('framework_node')
        self.width = width
        self.height = height
        self.keyCompressedPrev = 0U
        self.keyCompressed
        self.headerSize
        self.window
        self.context

        # needed for communicating with atlas_socket
        self.recvHandler = rospy.Publisher("SendBuffer", String, queue_size=10)
        self.sendHandler = rospy.Subscriber("RecvBuffer", String, ReceiveCallback)

    
    def binLoop() -> None : 
        while(not rospy.is_shutdown()): 
            pass 

    def binSend() -> list[char] :
        # original: recvSend()
        return []
    
    def binRecv(data:list[char]) -> None: 
        sizeOfC = len(self.commandsRos.nextSend) 
        sizeOfInt = 4 
        if(len(data) < sizeOfC + sizeOfInt): 
            return
        for i in range(len(data)):
            print("%d ", (int)data[i])
        print()
        self.commands.parse(data, sizeOfInt + sizeOfC);

        for i in range(sizeOfC): 
            c = data[sizeOfInt + i]
            if( c != 0 ): 
                for j in range(8): 
                    if(((c & (1<<j))) != 0): 
                        self.send(commandsRos.valAt(i*8+j))


    
    # initializes pygame object (sdl wrapper - sdl is the c++ equivalent)
    # creating a pygame screen, which is where the user input will go and the robot graphics will be shown
    def gl_setup(self, title:char, width:int, height:int): 
        pygame.init()
        screen_size = (width, height)
        screen = pygame.display.set_mode(screen_size)

    # destroy sdl objects by quitting pygame
    def __del__(self):
        # might need this before pygame.quit:
        # pygame.display.quit()
        
        pygame.quit()
        
        # then might need this after pygame.quit:
        # sys.exit()
    
    def ReceiveCallback(self, msg):
        self.recv(msg.data)



### https://www.pygame.org/docs/ref/key.html


    def handle_key_down(self, key): 
        if KeyMap.pygames_to_keys.contains(key):
            #original: keyCompressed |= (1<<(KeyMap.pygames_to_keys[key].value))
            keyCompressed += (1<<(KeyMap.pygames_to_keys[key].value))
            
    def handle_key_up(self): 
        if KeyMap.pygames_to_keys.contains(key):
            #original: keyCompressed &= ~(1<<(KeyMap.pygames_to_keys[key].value))
            keyCompressed -= (1<<(KeyMap.pygames_to_keys[key].value))




    def loop(self):
        #set up and start LoopFunc
        #sets up Manager to be able to run the Loop functoin
        self.commands.setup(False) 
        self.commandsRos.setup(False)

        #calls loop function 
        self.binLoop(False)

    #send is overloaded in the original cpp file
    # data:str[] and data:str
    def send(self, data:list[char] ):
        #rospy.loginfo(data)
        self.sendHandler.publish("".join(data))
        
    def recv(self, data:[char]):
        self.binRecv(data)



    def ReceiveCallback(self, msg:str) -> None:
        # void rdt::Manager::RecieveCallback(const std_msgs::String::ConstPtr& msg){
        #     recv(std::vector<char>(msg->data.begin(), msg->data.end()));
        # }
        # s = [msg.data.begin(), msg.data.end()]
        # recv(s)
        self.recv(msg)

    # this is literally blank lmao
    def SendImmediate(self):
        pass