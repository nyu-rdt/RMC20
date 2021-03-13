#!/usr/bin/env python

"""
atlas_socket.py

Takes in byte strings through the ROS topic 'send_buffer' and forwards them through the socket to 
the identical destination socket on the other side. Concurrently reads in data received through the 
socket and writes it to the ROS topic 'recv_buffer'. Additionally exchanges a constant stream of
dummy bytes to check that the connection is still alive. All actions performed at specified rate.

Used for communication between the Ground Control Station and the Server. Keyboard commands are
sent GCS->Server and robot data is sent Server->GCS.
"""

import rospy
from std_msgs.msg import String

import socket, json, os, time
import threading, Queue
import struct

# Global constants
G_TARGET_IP = "192.168.1.11"
#G_TARGET_IP = "127.0.0.1"
G_TARGET_PORT = 10010
G_LOCAL_IP = "192.168.1.10"
#G_LOCAL_IP = "127.0.0.1"
G_LOCAL_PORT = 10010
G_REPORT_RATE = 20                # Rate (in Hz) that connection is checked 
G_FAILSAFE_THRESHOLD = 60         # How many failed connections until ESTOP is triggered

# Maxsize <= 0 means infinite queue size
input_buffer = Queue.Queue(maxsize = -1)
output_buffer = Queue.Queue(maxsize = -1)

# Publishes output data
pub = rospy.Publisher("RecvBuffer", String, queue_size=10)

"""
Puts data from incoming ROS topic to input buffer; callback
"""
def send_buffer(data):
    global input_buffer
    if(len(data.data)>0):
        input_buffer.put(data.data)

# Subscribes to input data
rospy.Subscriber("SendBuffer", String, send_buffer)

ros_dead = False 

class AtlasSocket(threading.Thread):
    """
    Socket class initialization
    Has I/O buffers, target ip/port, local ip/port, report rate, failsafe threshold
    """
    def __init__(self, threadName, input_buffer, output_buffer, target_ip = '127.0.0.1', target_port = 10010, local_ip = '127.0.0.1', local_port = 10010, report_rate = 20, failsafe_th = 60):
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
        self.HEARTBEAT_PACKAGE = self.data_to_byte_string([0x4E, 0x52, 0x00])	#initialize the heartbeat package
        #a small amount of data that is sent to check if the connection is still alive
        self.last_data = self.HEARTBEAT_PACKAGE	#default value of last_data is heartbeat package
        super(AtlasSocket, self).__init__(name = threadName)
    
    """
    Trys to recieve data sent from the other atlas socket and stores it in output buffer 
    """
    def get_data(self): 
        receive_data = bytearray(2500)

        # Tries to read data from a socket
        try:
            receive_nbytes, receive_addr = self.conn.recvfrom_into(receive_data, 2048)	#read data from socket
        except:
            receive_nbytes = 0

        # If no data is received, increaase no response counter, This is the fail safee
        if receive_nbytes == 0:
            self.no_response_counter += 1
            # If the connection fails more than the threshold, break the connection
            if self.no_response_counter >= self.failsafe_th:
                rospy.loginfo("====== CONNECTION BROKE =====")
                self.no_response_counter = self.failsafe_th		
                self.conn_break = True	
                self.reset_socket()

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
        # Handle the no-response situation(reconnection) and send HEARTBEAT_PACKAGE to reconnect to the robot   
        if self.no_response_counter > 0:			
            self.conn.sendto(self.HEARTBEAT_PACKAGE, (self.target_ip, self.target_port))
            self.reconnection_flag = True

        # If there is at least one command need to be send in the input buffer(queue), send it
        elif not self.input_buffer.empty():
            # Clear the flag of reconnection
            if self.reconnection_flag:
                self.reconnection_flag = False  
            # Get new command from queue
            else:
                self.last_data = self.input_buffer.get()
            print(len(self.last_data), self.last_data)
            self.conn.sendto(self.last_data, (self.target_ip, self.target_port))
        # Send the heartbeat package if there is not data needed to be sent
        else: 
            self.conn.sendto(self.HEARTBEAT_PACKAGE, (self.target_ip, self.target_port))
        
    """
    Runs atlas socket; main loop
    """
    def run(self):
        global ros_dead
        ros_dead = rospy.is_shutdown()
        rate = rospy.Rate(self.report_rate) # 10hz

        # Run while ros topic is still active 
        while not ros_dead:
            ros_dead = rospy.is_shutdown()
            # If connection is unstable stop sending and recieving 
            if not self.hanging:
                self.get_data() 
                self.send_data()
            rate.sleep() # Sleep at self.report_rate frequency

        self.conn.close()
        
    """
    Initializes socket for starting the communication
    """
    def socket_init(self):
        if self.initialized:	                                            # If socket has been initialized, close it and initialize again
            self.conn.close()                                               # Close socket 
            self.initialized = False                                        # Clear initialized flag
        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)	# Initialize a socket object
            self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.conn.bind((self.local_ip, self.local_port))	            # Bind local ip and port
            self.conn.setblocking(0)
        except:
            self.conn.close()
            return
        self.initialized = True 	                                        # Set initialized flag

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
    def data_to_byte_string(self, data):
        s = ''
        for x in data:
            s += struct.pack('B',x)
        return s

    """
    Stop the program if the socket connection is stopped 
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
Retrieves data from output buffer topic and writes to the ROS topic
"""
def pub_thread():
    global output_buffer
    global ros_dead
    global pub

    # Get and send data until connection is established 
    while not ros_dead:
        try:
            data = output_buffer.get(timeout=1)
            string_data = str(data)
            pub.publish(data = string_data)
        except:
            pass
    return

if __name__ == '__main__':
    rospy.init_node("atlas_socket", anonymous=False)
    pub_thread = threading.Thread(target = pub_thread)
    pub_thread.start()
    sock = AtlasSocket("",input_buffer, output_buffer, target_ip = G_TARGET_IP, target_port = G_TARGET_PORT, local_ip = G_LOCAL_IP, local_port = G_LOCAL_PORT, report_rate = G_REPORT_RATE, failsafe_th = G_FAILSAFE_THRESHOLD)
    sock.socket_start()
    sock.run()
    pub_thread.join()
    #rospy.spin()




