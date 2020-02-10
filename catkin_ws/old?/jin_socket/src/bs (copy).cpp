// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string.h>
#include <pthread.h>
#include <net/if.h>
#include <pthread.h>
#define PORT 8080
#define NUM_THREADS 8


using namespace std;
int global_socket;

void Ros2Socket(std_msgs::String msg){
	send(global_socket,msg.data.c_str(),msg.data.size(),0);
}

void *Reciever(void* thing) {
	ros::NodeHandlePtr pointer = boost::make_shared<ros::NodeHandle>();
	char buffer[1024];
	while(ros::ok()){
	int valread = read( new_socket , buffer, 1024);
	printf("%s\n",buffer );
	ros::Publisher chatter_pub = pointer.advertise<std_msgs::String>("recvsend", 1000);
	ros::Rate loop_rate(10);
	std_msgs::String msg;
	std::stringstream ss;
	ss << buffer << endl;
	msg.data = ss.str();
	chatter_pub.publish(msg);
	}
}



void *Sender(void* thing) {
	ros::NodeHandlePtr pointer = boost::make_shared<ros::NodeHandle>();
	cout << "Mad Tiddies" <<endl;
	char buffer[1024];
	ros::Subscriber sub = pointer.subscribe("sendsend", 1000, Ros2Socket);
	ros::spin();
}

class Server1{
private:
    int server_fd, new_socket, valread;

    struct sockaddr_in myaddress; //STRUCT TO HOLD PACKET INFO THIS SERVER
    struct sockaddr_in address; //STRUCT TO HOLD PACKET INFO THIS ROBOT

    int opt = 1;
    int PORT_NUM = 8080; // Dummy PORT holder
    int addrlen = sizeof(address);

    char str[INET_ADDRSTRLEN];
    char *ip;  // Dummy IP holder

    pthread_t threads[NUM_THREADS];
	pthread_attr_t attr;
public:
    void Socket(){
        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }
        
		// Forcefully attaching socket to the port 8080
		// Forcefully attaching socket to the port 8080
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,&opt, sizeof(opt)))
        {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }
        myaddress.sin_family = AF_INET; //TYPE OF PROTCOL
	if(ip==NULL){
        	myaddress.sin_addr.s_addr = INADDR_ANY; // GET ANY FREE IP
	}
	else{
		inet_pton(AF_INET,ip, &myaddress.sin_addr.s_addr); //SET INOUT IP
	}
        myaddress.sin_port = htons( PORT_NUM ); //SET PORT
        
        // Forcefully attaching socket to the port 8080
        if (bind(server_fd, (struct sockaddr *)&myaddress,
                                    sizeof(myaddress))<0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
	// Listen to the port
        if (listen(server_fd, 8) < 0)
        {
            perror("listen");
            exit(EXIT_FAILURE);
        }
	// If correctly connected
        int i = 0;
        printf("The Server is on IP:%s Port:%d \n",ip,ntohs(myaddress.sin_port));
        while(true){
	        if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
	                        (socklen_t*)&addrlen))<0)
	        {
	            perror("accept");
	            exit(EXIT_FAILURE);
	        }
	        else{
				cout << "Socket() : creating thread, " << i << endl;
				cout << "Socket() : creating socketid, " << new_socket << endl;
				int rc = pthread_create(&threads[i], &attr, Reciever,(void*)i++);
				if (rc) {
				 cout << "Error:unable to create thread," << rc << endl;
				 exit(-1);
				}
				cout << "Socket() : creating thread, " << i << endl;
				cout << "Socket() : creating socketid, " << new_socket << endl;
				rc = pthread_create(&threads[i], &attr, Sender,(void*)i++);
				if (rc) {
				 cout << "Error:unable to create thread," << rc << endl;
				 exit(-1);
				}
				global_socket = new_socket;

	   //      }
	        printf("The Server is on IP:%s Port:%d \n",ip,ntohs(myaddress.sin_port));
		printf("The Client is on IP:%s Port:%d \n",inet_ntoa(address.sin_addr),ntohs(address.sin_port));
	        }
	    }
    }


    Server1(char* ip1, int port){
    ip = ip1;
    PORT_NUM = port;

	// Initialize and set thread joinable
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    
    Socket();
    //threads_end();
    }   
};

int main(int argc, char **argv){
	char* ip1 = "127.0.0.1";
	int port1 = 8008;
	if (argc > 3) {
		fprintf(stderr, "Too much Input\n");
		exit(1);
	}
	else if (argc == 3) {
		port1 = atoi(argv[1]);
		ip1 = (char*) argv[2];
	}
	else if (argc == 2) {
		port1 = atoi(argv[1]);
	}
	ros::init(argc, argv, "socket");
   	Server1 RDT(ip1,port1);   
}
