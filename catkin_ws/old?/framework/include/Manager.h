#pragma once
#include "Keyboard.h"
#include "JumpTable.h"
#include "ROSTable.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "framework/IntStr.h"

#include <SDL2/SDL.h>

class Manager;

namespace rdt{
    class Manager{
    private:
        int width;
        int height;
        SDL_Window* window;
        SDL_GLContext context;

        unsigned int keyCompressedPrev;
        unsigned int keyCompressed;

        ros::NodeHandle* handler = nullptr;
        ros::Subscriber recvHandler;
        ros::Publisher sendHandler;

        bool sender;
        int headerSize;

        void gl_setup(const char* title, int width, int height);
        void handle_key_down(SDL_Keysym* key);
        void handle_key_up(SDL_Keysym* key);

        void (rdt::Manager::*LoopFunc)();
        std::vector<char> (rdt::Manager::*SendFunc)();
        void (rdt::Manager::*RecvFunc)(const std::vector<char>&);

        void sendLoop();
        std::vector<char> sendSend();
        void sendRecv(const std::vector<char>&);

        void recvLoop();
        std::vector<char> recvSend();
        void recvRecv(const std::vector<char>&);

        void RecieveCallback(const std_msgs::String::ConstPtr& msg);

        //on reciever side only, force ros data to be sent across network
        bool SendImmediate(framework::IntStr::Request& req, framework::IntStr::Response& res);

    public:
        FunctionTable commands;
        RosTable commandsRos;

        Manager(int argc, char** argv, const char* title = "RDT Command Framework", int width = 640, int height = 480);
        ~Manager();

        void loop();

        void send(const std::vector<char>& data);
        void send(const std::string& data);
        void recv(const std::vector<char>& data);

    };
}
