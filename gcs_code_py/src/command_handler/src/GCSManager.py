# This file acts as the GCS Manager, is the NUC SENDING commands from GCS to Arena
# Contains all GCS functions from original Manager.py, main functionality is SENDING
# While sending is the main purpose, GCS also might receive feedback from BIN
# https://github.com/nyu-rdt/RMC20/blob/ground_control_station/gcs_code/src/framework/src/Manager.cpp


#!/usr/bin/env python
# import filenames-interpreter and user-input libraries
import sys, pygame, rospy, KeyMap

from std_msgs.msg import String
from jumptable import *
from ROSTable import *


# pop up status indicator window for connection status 
# green circle for good connection, red for bad
class GCSManager:
    def __init__(self, title="  RDT Command Framework", width=640, height=480):
        
        self.commands = FunctionTable() 
        self.commandsRos = RosTable() 
        
        # ros init with command line arguments, needs to be finished. possible line of code:
        rospy.init_node('gcs_manager')
        self.width = width
        self.height = height
        self.keyCompressedPrev = 0
        self.keyCompressed = 0
        self.headerSize = 4
        #self.window
        #self.context

        self.gl_setup(title, width, height)

        # needed for communicating with atlas_socket
        self.sendHandler = rospy.Publisher("SendBuffer", String, queue_size=10)
        self.recvHandler = rospy.Subscriber("RecvBuffer", String, self.ReceiveCallback)

    def gcsLoop(self): 
        # original: sendLoop()
        '''
        handels key presses & publishes to ROS
        not sure if se
        '''
        #publish_data = rospy.Publisher(self.keyCompressed, String)
        
        print("Starting: \n")
        delta = 1000 #1 second
        curr = pygame.time.get_ticks() 
        threshold = curr + delta
        while(not rospy.is_shutdown()):
            #current time 
            curr = pygame.time.get_ticks() 
            #check for if there is a keyboard click 
            for event in pygame.event.get(): #gets a queue of events
                if event.type == pygame.NOEVENT: 
                    break
                elif event.type == pygame.QUIT: return
                #if a key is pressed
                elif event.type == pygame.KEYDOWN:
                    self.handle_key_down(event.key)
                    currentData = self.gcsSend() 
                    #sends data s
                    self.send(currentData)
                    threshold = curr+delta
                #if a key was released 
                elif event.type == pygame.KEYUP:
                    self.handle_key_up(event.key)
                    currentData = self.gcsSend() 
                    #sends data 
                    self.send(currentData)
                    threshold = curr+delta
                
            # if there is no keyboard command for threshold length (currently 1 second) delta variable, send the ROStable info
            if(threshold<curr): 
                threshold = curr+delta 
                #4 is the size of an integer in bytes 
                header = [0 for _ in range(4+len(self.commandsRos.nextSend))]
                for i in range(4):
                    header[i] = curr & 0xFF 
                    curr = curr >> 8 
                for i in range(len(self.commandsRos.nextSend)): 
                    header[4+i] = self.commandsRos.nextSend[i]
                self.send(header);

    
        #https://www.pygame.org/docs/ref/event.html#pygame.event.get
                

        
    
    def gcsSend(self):
        #First 4 index of Header represent the time it was sent, the rest of header repersents the command being sent 
        #this will only work for 71582 minutes before conflicts or 2^32 miliseconds 
        # original: sendSend()
        if(self.commands.has_update(self.keyCompressed ^ self.keyCompressedPrev)): 
            header = [] 
            tick = pygame.time.get_ticks()  
            shift = 4 #size of tick in bytes. Tick is an int so 4 bytes
            #there might be a conversion error in python because python ints are 24 bytes

            header = [0 for _ in range(shift+ len(self.commandsRos.nextSend))]
            for i in range(0, shift): 
                header[i] = tick & 0xFF
                tick = tick >> 8 
            # itterates through the rest of the array len(self.commandsRos.nextSend)
            for i in range(shift, len(header)): 
                header[i] = self.commandsRos.nextSend[i-shift]  
            self.commands.encode(self.keyCompressed, self.keyCompressedPrev, header)
            self.keyCompressedPrev = self.keyCompressed

            return header 
        return [] 
            

    def gcsRecv(self, data):
        # original: sendRecv()
        if(len(data) == 0):
            return 
        tmp1 = int(data[0]) 
        #might need to change the str part idk what htis does std::string(data.begin()+1,data.end())
        self.commandsRos.decode(tmp1, "".join([chr(byte) for byte in data]) );
        tmp2 = tmp1 >> 3 
        tmp1 -= tmp2 << 3  

        self.commandsRos.nextSend[tmp2] &= ~(1<<tmp1);

    
    # initializes pygame object (sdl wrapper - sdl is the c++ equivalent)
    # creating a pygame screen, which is where the user input will go and the robot graphics will be shown
    def gl_setup(self, title, width, height): 
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
        if key in KeyMap.pygame_to_keys:
            #original: keyCompressed |= (1<<(KeyMap.pygame_to_keys[key].value))
            self.keyCompressed += (1<<(KeyMap.pygame_to_keys[key].value))
            
    def handle_key_up(self, key): 
        if key in KeyMap.pygame_to_keys:
            #original: keyCompressed &= ~(1<<(KeyMap.pygame_to_keys[key].value))
            self.keyCompressed -= (1<<(KeyMap.pygame_to_keys[key].value))


    def loop(self):
        #sets up Manager to be able to run the Loop functoin
        self.commands.setup(True) 
        self.commandsRos.setup(True)

        #calls loop function 
        self.gcsLoop()

    #send is overloaded in the original cpp file
    # data:str[] and data:str
    def send(self, data):
        #rospy.loginfo(data)
        print("data:", data)
        self.sendHandler.publish("".join([chr(byte) for byte in data]))
        
    def recv(self, data):
        self.gcsSend(data)


