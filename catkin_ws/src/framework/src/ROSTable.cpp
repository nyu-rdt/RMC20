#include "ROSTable.h"

rdt::RosTable::RosTable() {
    idMap = std::map<std::string,int>();
    data = std::vector<RosFunction>();
    nextSend = std::vector<char>();
}

rdt::RosTable::~RosTable(){
}

void rdt::RosTable::AddBroadcast(
            const std::string& identifier,
            void(*setup)(bool),
            void(*decoder)(const std::string&))
{
    if(idMap.find(identifier) != idMap.end()){
        printf("%s already in use", identifier.c_str());
        return;
    }

    int curr = data.size();
    data.push_back(RosFunction());
    data[curr].setup = setup;
    data[curr].decoder = decoder;
    if(data[curr].lock == nullptr){
        data[curr].lock = new std::mutex();
    }
    idMap[identifier] = curr;

    ++curr;
    int tmp = curr/8;
    if(tmp*8 < curr){ ++ tmp;}
    if(tmp > nextSend.size()){
        printf("adding ros buffer\n");
        nextSend.push_back((char)0);
    }
}

void rdt::RosTable::setup(bool sender){
    for(RosFunction& func: data){
        if(func.setup != nullptr){func.setup(sender);}
    }
}

std::string rdt::RosTable::valAt(int index){
    if(index < 0 || index >= data.size()){return "";}
    //printf("getting from %d\n", index);
    std::lock_guard<std::mutex> lock(*data[index].lock);
    std::stringstream ss;
    ss<<(char)index<<data[index].data;
    std::string str = ss.str();
    //printf("%s", str.c_str());
    return str;
}

void rdt::RosTable::decode(int index, const std::string& msg){
    if(index < 0 || index >= data.size()){return;}
    if(data[index].decoder!=nullptr){
        data[index].decoder(msg);
    }
}

bool rdt::RosTable::GetID(framework::StrInt::Request& reqID, framework::StrInt::Response& resID){
    std::map<std::string, int>::iterator iter = idMap.find(reqID.data);
    if(iter == idMap.end()){ resID.data = -1; }
    else{ resID.data = iter->second; }
    return true;
}

bool rdt::RosTable::SetData(framework::IntStr::Request& req, framework::IntStr::Response& res){
    //printf("called set %d\n",req.ID);
    if(req.ID < 0 || req.ID >= data.size()){ return true;}
    std::lock_guard<std::mutex> lock(*data[req.ID].lock);
    data[req.ID].data = req.data;
    //printf("Data %s\n",req.data.c_str());
    res.success = true;
    return true;
}

bool rdt::RosTable::GetData(framework::IntStr::Request& req, framework::IntStr::Response& res){
    if(req.ID < 0 || req.ID >= data.size()){return false;}
    //printf("size %d\n", nextSend.size());
    int tmp1,tmp2;
    tmp1 = req.ID;
    //printf("tmp1 %d\n", tmp1);
    tmp2 = tmp1 >> 3;
    //printf("tmp2 %d\n", tmp2);
    tmp1 -= tmp2 << 3;
    //printf("tmp1 %d\n", tmp1);
    nextSend[tmp2] |= (char)(1<<tmp1);
    res.success = true;
    //printf("done callback\n");
    return true;
}

