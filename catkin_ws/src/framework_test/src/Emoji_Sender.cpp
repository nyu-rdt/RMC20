#include "ros/ros.h"
#include "std_msgs/String.h"
#include "framework/IntStr.h"
#include "framework/StrInt.h"

int setUpID(ros::NodeHandle& n){
    ros::ServiceClient client = n.serviceClient<framework::StrInt>("get_ROS_ID");
    framework::StrInt getID;
    getID.request.data = "EMOJI";

    while(!client.call(getID)){ros::spinOnce();}
    return getID.response.data;
}

void listenToEmojis(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("%s",msg->data.c_str());    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "emoji_gcs");
    ros::NodeHandle n("s");
    ros::Rate loop_rate(10);
    
    ros::NodeHandle emoji("emoji");

    framework::IntStr id;
    id.request.ID = setUpID(n);

    ros::ServiceClient client = n.serviceClient<framework::IntStr>("req_ROS_Data");
    ros::Subscriber eSub = emoji.subscribe("emoji", 1, listenToEmojis);

    while (ros::ok()){
        client.call(id);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
