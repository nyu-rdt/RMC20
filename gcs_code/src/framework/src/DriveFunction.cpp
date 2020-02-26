#include "std_msgs/String.h"

#include "FunctionList.h"

// TODO: shift key for extra speed?
// Decoder for robot driving commands. W key moves forward

//Kiersten did the encoding/decoding
//She tried really hard

//ros::NodeHandle pointer;
ros::NodeHandle pointer;
ros::Publisher DriveFunction_Pub = pointer.advertise<std_msgs::String>("server/heartbeat", 1000);
std::vector<rdt::Keyboard> driveKey = {rdt::Keyboard::W, rdt::Keyboard::A, rdt::Keyboard::S, rdt::Keyboard::D};

std::vector<char> driveEncoder(const std::vector<bool>& input) { //Input is true or false for each key
    std::vector<char> out;
		// can still implement two keys being pressed at the same type
    if (input[0]){out[0] = 0;} //w - forwards 
		if (input[1]){out[0] = 1;} //a - left
		if (input[2]){out[0]= 2;} //s - backwards
		if (input[3]){out[0]= 3;} //d - right

		return out; 
}

void driveDecoder(const std::vector<char>& data){ //Data is the result of the encoder
		//will receive a 7-char string that will be broken down into separate commands
		//data[0]: 0 = forwards/ backwards, 1 = rotating, 2 = arm movement
		//data[1]: 0  = forwards, 1 = backwards
		//data[2, 3]: 0 - 99; = speed
		
		std::string command;	

//will implement turning after the demonstration, this code only allows for the robot
		//to move forwards or backwards at a set speed
		//first part of the string represents the forward-backwards drive mode, 0 is reserved for 
		//estop
		if(data[0] == 0){ //go forwards
			command = "10050000"; //speed set to 50
		}
		//else if(data[0] == 1]){ //go left

		//}
		else if(data[0] == 2){ //go backwards
			command = "10150000"; //speed set to 50
		}
		//else if(data[0] == 3]){ //go right
		//}
		std_msgs::String com;
		com.data = command;
		DriveFunction_Pub.publish(com);
		ros::spinOnce();
}

void driveSetup(bool sender){} //Prepares the ROS Topic here
				//sender == 0 then receive, == 1 then send
         //Dan, Charcles told me if this doesn't work then make the pointer in global above the publisher
void driveCleanup(bool sender){} //Do nothing like Jin
