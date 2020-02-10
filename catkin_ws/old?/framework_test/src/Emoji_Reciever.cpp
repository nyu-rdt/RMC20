#include "ros/ros.h"
#include "std_msgs/String.h"
#include "framework/IntStr.h"
#include "framework/StrInt.h"

#include <string>
#include <vector>

int setUpID(ros::NodeHandle& n){
    ros::ServiceClient client = n.serviceClient<framework::StrInt>("get_ROS_ID");
    framework::StrInt getID;
    getID.request.data = "EMOJI";

    while(!client.call(getID)){ros::spinOnce();}
    return getID.response.data;
}

std::vector<std::string> emos = {"^_^","-_-",":D",":)",":(","XD",";_;","T_T","TT_TT",";P",":P"};

int main(int argc, char** argv){
    ros::init(argc, argv, "emoji_bin");
    ros::NodeHandle n("r");
    ros::Rate loop_rate(10);

    framework::IntStr id;
    id.request.ID = setUpID(n);
    ros::ServiceClient client = n.serviceClient<framework::IntStr>("set_ROS_Data");    
    printf("starting %d\n", id.request.ID);
    while(ros::ok){
        id.request.data = emos[rand()%emos.size()];
        client.call(id);
        ros::spinOnce();
        //printf("Worked\n");
        loop_rate.sleep();
    }
    
}
