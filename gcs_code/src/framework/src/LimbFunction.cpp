#include "FunctionList.h"

// TODO: shift key for extra speed?
// Decoder for robot driving commands. W key moves forward

//ros::NodeHandle pointer;
ros::Publisher DriveFunction_Pub = pointer.advertise<std_msgs::String>("LimbFunction_Pub", 1000);
std::vector<rdt::Keyboard> driveKey = { rdt::Keyboard::U, rdt::Keyboard::J};
//U is for moving the 'arms' up, and J is for down
std::vector<char> driveEncoder(const std::vector<bool>& input) { //Input is true or false for each key
    std::vector<char> out[1];
    
    if (input[0]){out[0] = 1;} //u - up
    else if (input[1]){out[0] = 2;} //j - down
    else{out[0] = 0;}
    return out; 
}

void driveDecoder(const std::vector<char>& data){ 
    //Data is the result of the encoder
    //Data will either be 0, or 1.
    //0 = not moving
    //1 = up
    //2 = down 
		
		std::string command;		
		
		if(data[0] == 0){ //don't move anything
			command = "00000000";
		}
		else if(data[0] == 1]){ //go up
			command = "01500000";
		}
		else if(data[0] == 3]){ //go down
			command = "02500000";
		}
		std_msgs::String com;
		com.data = command;
		DriveFunction_Pub.publish("robotCmds/drive", com.data.c_str());
		ros::spinOnce();
}

void driveSetup(bool sender); //Prepares the ROS Topic here
				//sender == 0 then receive, == 1 then send
        ros::NodeHandle pointer; //Dan, Charcles told me if this doesn't work then make the pointer in global above the publisher
void driveCleanup(bool sender); //Do nothing like Jin
