# convert Manager to python
# https://github.com/nyu-rdt/RMC20/blob/ground_control_station/gcs_code/src/framework/src/Manager.cpp


#!/usr/bin/env python
# import filenames-interpreter and user-input libraries
import sys, pygame, rospy, KeyMap

from std_msgs.msg import String


# pop up status indicator window for connection status 
# green circle for good connection, red for bad
class Manager:
    def __init__(self, title="  RDT Command Framework":str, width=640:int, height=480:int):
        
        self.commands = FunctionTable() 
        self.commandsRos = RosTable() 
        
        # ros init with command line arguments, needs to be finished. possible line of code:
        # rospy.init_node('framework_node')
        self.width = width
        self.height = height
        self.isGcs = True #gcs == ground control station, if false == NUC is on the bin originally sender
        self.keyCompressedPrev = 0U
        self.keyCompressed
        self.headerSize
        self.window
        self.context

        # needed for communicating with atlas_socket
        self.recvHandler = rospy.Publisher("SendBuffer", String, queue_size=10)
        self.sendHandler = rospy.Subscriber("RecvBuffer", String, ReceiveCallback)

        # setting function pointers
        if (self.isGcs):
            # this file is running on the gcs
            #orginially was send[name] in cpp file
            gl_setup(title, width, height)
            self.loopFunc = gcsLoop 
            self.sendFunc = gcsSend
            self.recvFunc = gcsRecv 
        else:
            # this file is running on the bin
            #orginially was recv[name] in cpp file
            self.loopFunc = binLoop
            self.sendFunc = binSend
            self.recvFunc = binRecv

    def gcsLoop() -> None: 
        # original: sendLoop()
        '''
        handels key presses & publishes to ROS
        not sure if se
        '''
        publish_data = rospy.Publisher(self.keyCompressed, String)
        
        print("Starting: \n")
        delta = 1000 #1 second
        curr = pygame.time.get_ticks() 
        threshold = curr + delta
        while(not rospy.is_shutdown()):
            #current time 
            curr = pygame.time.get_ticks() 
            #check for if there is a keyboard click 
            for events in pygames.event.get(): #gets a queue of events
                if event.type = pygame.NOEVENT: 
                    break
                elif event.type == pygame.QUIT: return
                #if a key is pressed
                elif event.type == pygame.KEYDOWN:
                    self.handle_key_down(event.key)
                    currentData = self.sendFunc() 
                    #sends data 
                    self.send(currentData)
                    threshold = curr+delta
                #if a key was released 
                elif event.type == pygame.KEYUP:
                    self.handle_key_up(event.key)
                    currentData = self.sendFunc() 
                    #sends data 
                    self.send(currentData)
                    threshold = curr+delta
                
            # if there is no keyboard command for threshold length (currently 1 second) delta variable, send the ROStable info
            if(threshold<curr): 
                threshold = curr+delta 
                #4 is the size of an integer in bytes 
                header = [0 for _ in range(4+len(self.commandsRos.nextSend)]
                for i in range(4): s    s
                    header[i] = str(curr & 0xFF) 
                    cur = cur >> 8 
                for i in range(len(self.commandsRos.nextSend)): 
                    header[4+i] = self.commandsRos.nextSend[i]
                self.send(header);

    
        #https://www.pygame.org/docs/ref/event.html#pygame.event.get
                

        
    
    def gcsSend() -> list[char]:
        #First 4 index of Header represent the time it was sent, the rest of header repersents the command being sent 
        #this will only work for 71582 minutes before conflicts or 2^32 miliseconds 
        # original: sendSend()
        if(self.commands.hasUpdate(self.keyCompressed ^ self.keyCompressedPrev)): 
            header = [] 
            tick = pygame.time.get_ticks()  
            shift = 4 #size of tick in bytes. Tick is an int so 4 bytes
            #there might be a conversion error in python because python ints are 24 bytes

            header = [0 for _ in range(shift+ len(self.commandsRos.nextSend)]
            for i in range(0, shift): 
                header[i] = str(tick & 0xFF)
                tick = tick >> 8 
            # itterates through the rest of the array len(self.commandsRos.nextSend)
            for i in range(shift, len(header)): 
                header[i] = self.commandsRos.nextSend[i-shift]  
            self.commands.encode(self.keyCompressed, self.keyCompressedPrev, header)
            self.keyCompressedPrev = self.keyCompressed

            return header 
        return [] 
            

    def gcsRecv(data:list[char]):
        # original: sendRecv()
        if(len(data) == 0):
            return 
        tmp1 = int(data[0]) 
        #might need to change the str part idk what htis does std::string(data.begin()+1,data.end())
        self.commandsRos.decode(tmp1, "".join(data) );
        tmp2 = tmp1 >> 3 
        tmp1 -= tmp2 << 3  

        self.commandsRos.nextSend[tmp2] &= ~(1<<tmp1);


    
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
        self.commands.setup(self.isGcs) 
        self.commandsRos.setup(self.isGcs)

        #calls loop function 
        self.LoopFunc()

    #send is overloaded in the original cpp file
    # data:str[] and data:str
    def send(self, data:list[char] ):
        #rospy.loginfo(data)
        self.sendHandler.publish("".join(data))
        
    def recv(self, data:[char]):
        self.recvfunc(data)



    def RecieveCallback(self, msg:str) -> None:
        # void rdt::Manager::RecieveCallback(const std_msgs::String::ConstPtr& msg){
        #     recv(std::vector<char>(msg->data.begin(), msg->data.end()));
        # }
        # s = [msg.data.begin(), msg.data.end()]
        # recv(s)
        self.recv(msg)

    # this is literally blank lmao
    def SendImmediate(self):
        pass