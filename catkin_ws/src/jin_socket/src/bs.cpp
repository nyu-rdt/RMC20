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
	ros::NodeHandle pointer;
	ros::Publisher chatter_pub = pointer.advertise<std_msgs::String>("send", 1000);
	ros::Subscriber sub = pointer.subscribe("send", 1000, Ros2Socket);
   	//Server1 RDT(ip1,port1);   
}
