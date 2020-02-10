#pragma once
#include <map>
#include <vector>
#include <string>
#include <mutex>

#include "ros/ros.h"
#include "framework/IntStr.h"
#include "framework/StrInt.h"

class RosTable;
class Manager;

namespace rdt {

    struct RosFunction{
        std::string data;
        void(*setup)(bool) = nullptr;
        void(*decoder)(const std::string&) = nullptr;
        std::mutex* lock;

        RosFunction():
            data("")
        {
            setup = nullptr;
            decoder = nullptr;
            lock = new std::mutex();
        }

        RosFunction(const RosFunction& o){
            data = o.data;
            setup = o.setup;
            decoder = o.decoder;
            lock = new std::mutex();
        }

        RosFunction& operator=(const RosFunction& o){
            data = o.data;
            setup = o.setup;
            decoder = o.decoder;
            return *this;
        }

        ~RosFunction(){
        }
    };

    class RosTable {
    friend class Manager;
    public:
        RosTable();
        ~RosTable();

        /**
        setup and decoder is only called if the node is a sender
        */
        void AddBroadcast(
            const std::string& identifier,
            void(*setup)(bool),
            void(*decoder)(const std::string&)
        );

    private:
        std::map<std::string,int> idMap;
        std::vector<RosFunction> data;
        std::vector<char> nextSend;

        std::string valAt(int index);
        void decode(int index, const std::string& data);

        void setup(bool sender);

        bool GetID(framework::StrInt::Request& reqID, framework::StrInt::Response& resID);
        bool SetData(framework::IntStr::Request& req, framework::IntStr::Response& res);
        bool GetData(framework::IntStr::Request& req, framework::IntStr::Response& res);
    };
}
