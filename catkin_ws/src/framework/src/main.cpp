#include "Manager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "FunctionList.h"

std::vector<char> movementE(const std::vector<bool>& input){
    std::vector<char> out;
    out.push_back(0);
    if(input[0]){out[0]|=1;}
    if(input[1]){out[0]|=2;}
    if(input[2]){out[0]|=4;}
    if(input[3]){out[0]|=8;}
    return out;
}

void movementD(const std::vector<char>& data){
    //printf("%d", (int)data[0]);
    std::string s;
    if(data[0]&1){ s+="W"; }
    if(data[0]&2){ s+="A"; }
    if(data[0]&4){ s+="S"; }
    if(data[0]&8){ s+="D"; }
    ROS_INFO("%s", s.c_str());
}

ros::NodeHandle* emojiHandler;
ros::Publisher emojiPub;

void emojiSetup(bool sender){
    if(sender){
        emojiHandler = new ros::NodeHandle("emoji");
        emojiPub = emojiHandler->advertise<std_msgs::String>("emoji", 1);
    }
}

void emojiDecoder(const std::string& data){
    std_msgs::String msg;
    msg.data = data;
    emojiPub.publish(msg);
}

int main(int argc, char** argv){
    rdt::Manager manager(argc, argv);
    manager.commands.insert(1, {rdt::Keyboard::W,rdt::Keyboard::A,rdt::Keyboard::S,rdt::Keyboard::D},
    movementE, movementD);

    manager.commands.insert(1, jumpK, jumpE, jumpD, jumpS, jumpC);

    manager.commandsRos.AddBroadcast("slot1",nullptr,nullptr);
    manager.commandsRos.AddBroadcast("slot2",nullptr,nullptr);
    manager.commandsRos.AddBroadcast("slot3",nullptr,nullptr);
    manager.commandsRos.AddBroadcast("slot4",nullptr,nullptr);
    manager.commandsRos.AddBroadcast("slot5",nullptr,nullptr);
    manager.commandsRos.AddBroadcast("slot6",nullptr,nullptr);
    manager.commandsRos.AddBroadcast("slot7",nullptr,nullptr);
    manager.commandsRos.AddBroadcast("EMOJI", emojiSetup,emojiDecoder);
    manager.commandsRos.AddBroadcast("slot8",nullptr,nullptr);

    manager.loop();
    return 0;
}
