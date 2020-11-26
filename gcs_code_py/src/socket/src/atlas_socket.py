#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import socket, json, os, time
import threading, Queue
import struct

# maxsize <= 0 means infinite queue size
inputBuffer = Queue.Queue(maxsize = -1)
outputBuffer = Queue.Queue(maxsize = -1)

#publishes output data
pub = rospy.Publisher("RecvBuffer", String, queue_size=10)

"""
"""
def SendBuffer(data):
    global inputBuffer
    if(len(data.data)>0):
        inputBuffer.put(data.data)

#subscribes input data
rospy.Subscriber("SendBuffer", String, SendBuffer)

ros_dead = False 

class AtlasSocket(threading.Thread):
    """
        socket class initialization
        has an I/O buffer, target ip/port, local ip/port, report rate, failsafe_th
    """
    def __init__(self, threadName, input_buffer, output_buffer, target_ip = '127.0.0.1', target_port = 10010, local_ip = '127.0.0.1', local_port = 10010, report_rate = 20, failsafe_th = 5):
        self.input_buffer = input_buffer
        self.output_buffer = output_buffer
        self.target_ip = target_ip
        self.target_port = target_port
        self.local_ip = local_ip
        self.local_port = local_port
        self.initialized = False
        self.no_response_counter = 0
        self.conn_break = False
        self.hanging = True 
        self.failsafe_th = failsafe_th #Fail Safe Threshold, the amount of time the connection is allowed to fail 
        self.report_rate = report_rate
        self.reconnection_flag = False 
        self.HEARTBEAT_PACKAGE = self.data2bytestring([0x4E, 0x52, 0x00])	#initialize the heartbeat package
        #a small amount of data that is sent to check if the connection is still alive
        self.last_data = self.HEARTBEAT_PACKAGE	#default value of last_data is heartbeat package
        super(AtlasSocket, self).__init__(name = threadName)
    
    """
    Trys to recieve data sent from the other atlas socket and stores it in output buffer 
    """
    def get_data(self): 
        receive_data = bytearray(2500)
        #tries to read data from a socket
        try:
            receive_nbytes, receive_addr = self.conn.recvfrom_into(receive_data, 2048)	#read data from socket
        except:
            receive_nbytes = 0
        #If no data is received, increaase no response counter, This is the fail safee
        if receive_nbytes == 0:
            self.no_response_counter += 1
            #if the connection fails more than the threshold, break the connection
            if self.no_response_counter >= self.failsafe_th:
                self.no_response_counter = self.failsafe_th		
                self.conn_break = True	
                self.reset_socket()
                #receive_dict['type'] = 'connection_break'	#make a 'connection_break' data to inform the main thread
                #self.output_buffer.put(receive_data)		#put dict into output buffer(queue)
        #If data received, put the bytes into ouput_buffer       
        else:	
            self.no_response_counter = 0	
            self.conn_break = False
            print("Recieved", receive_data[0:receive_nbytes])
            self.output_buffer.put(receive_data[0:receive_nbytes])
    
    """
    Send data from the rostopic, SendBuffer to the heartbeat_package
    """
    def send_data(self):
        #handle the no-response situation(reconnection) and send HEARTBEAT_PACKAGE to reconnect to the robot   
        if self.no_response_counter > 0:			
            self.conn.sendto(self.HEARTBEAT_PACKAGE, (self.target_ip, self.target_port))
            self.reconnection_flag = True
        #if there is at least one command need to be send in the input buffer(queue), send it
        elif not self.input_buffer.empty():
            #clear the flag of reconnection
            if self.reconnection_flag:
                self.reconnection_flag = False  
            #get new command from queue
            else:
                self.last_data = self.input_buffer.get()
            print(len(self.last_data), self.last_data)
            self.conn.sendto(self.last_data, (self.target_ip, self.target_port))
        #send the heartbeat package if there is not data needed to be sent
        else: 
            self.conn.sendto(self.HEARTBEAT_PACKAGE, (self.target_ip, self.target_port))
        
    """
    Runs atlas socket 
    """
    def run(self):
        global ros_dead
        ros_dead = rospy.is_shutdown()
        rate = rospy.Rate(self.report_rate) #10hz
        #run while ros topic is still active 
        while not ros_dead:
            ros_dead = rospy.is_shutdown()
            #if connection is unstable stop sending and recieving 
            if not self.hanging:
                self.get_data() 
                self.send_data()
            rate.sleep() #sleep at self.report_rate frequency
        self.conn.close()
        
    """
        socket_init(self)
        initializes socket for starting the communication
    """
    def socket_init(self):
        if self.initialized:	#if socket has been initialized, close it and initialize again
            self.conn.close()   #close socker 
            self.initialized = False #clear initialized flag
        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)	#initialize a socket object
            self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.conn.bind((self.local_ip, self.local_port))	#bind local ip and port
            self.conn.setblocking(0)
        except:
            self.conn.close()
            return
        self.initialized = True 	#set initialized flag

    """
    Reset socket to its original state if there is no connection
    """
    def reset_socket(self): 
        global ros_dead
        self.socket_stop()     
        self.socket_start()
        while(not self.initialized and not ros_dead):
            self.socket_start()
        self.conn_break = False
        self.no_response_counter = 0
    """
    Change the Target info
    """
    def change_target(self, target_ip, target_port):
        self.target_ip = target_ip
        self.target_port = target_port

    """
    Change the Local info
    """
    def change_local(self, local_ip, local_port):	
        self.local_ip = local_ip
        self.local_port = local_port
        self.socket_init()

    """
    Change the Report Rate
    """
    def change_report_rate(self, report_rate):
        self.report_rate = report_rate

    """
    Changes the Failure Threshold
    """
    def change_failsafe_threshold(self, failsafe_th):	
        self.failsafe_th = failsafe_th

    """
    Convert data from an integer list into a bytestring e.g.[0x01,0x02,0x03] -> '\x01\x02\x03'
    """
    def data2bytestring(self, data):
        s = ''
        for x in data:
            s += struct.pack('B',x)
        return s

    """
        socket_stop(self)
        stops socket communication, hanging is True

        socket_start(self) 
        initializes socket, hanging is False
    
    """

    """
    stop the program if the socket connection is stopped 
    """
    def socket_stop(self):
        self.hanging = True
    """
    Initializes the socket and starts the program
    """
    def socket_start(self):
        self.socket_init()
        self.hanging = False

"""
    retrieves data from output buffer topic and writes to the ROS topic
"""
def pubThread():
    global outputBuffer
    global ros_dead
    global pub

    #get and send data until connection is established 
    while not ros_dead:
        try:
            data = outputBuffer.get(timeout=1) #retrieve data from output buffer topic
            string_data = str(data) #translate everything to string
            #rospy.loginfo(data)
            pub.publish(data = string_data) #publishes data to the ROS topic
        except:
            pass
        #rospy.loginfo("There are not strangers to love. You know the rules and so do I ~")
    return

if __name__ == '__main__':
    rospy.init_node("atlas_socket", anonymous=False)
    pubThread = threading.Thread(target = pubThread)
    pubThread.start()
    sock = AtlasSocket("",inputBuffer, outputBuffer, target_ip = '192.168.1.2' , local_ip = '192.168.1.15')
    sock.socket_start()
    sock.run()
    pubThread.join()
    #rospy.spin()




