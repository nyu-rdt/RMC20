#include "Manager.h"
#include <thread>
#include <sstream>

void rdt::Manager::gl_setup(const char* title, int width, int height){
    if(SDL_Init( SDL_INIT_VIDEO ) < 0) {
        ROS_INFO("SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
        exit(0);
    }
    window = SDL_CreateWindow( title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN );
    if(window == NULL) {
        ROS_INFO("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        exit(0);
    }
    context = SDL_GL_CreateContext(window);
}

void rdt::Manager::handle_key_down(SDL_Keysym* key){
    std::map<int,Keyboard>::const_iterator iter = SDLtoRDT.find(key->sym);
    if(iter != SDLtoRDT.end()){
        keyCompressed |= 1<<((int)iter->second);
    }
    //printf("%08x\n",keyCompressed);
}

void rdt::Manager::handle_key_up(SDL_Keysym* key){
    std::map<int,Keyboard>::const_iterator iter = SDLtoRDT.find(key->sym);
    if(iter != SDLtoRDT.end()){
        keyCompressed &= ~(1<<((int)iter->second));
    }
    //printf("%08x\n",keyCompressed);
}

void rdt::Manager::sendLoop(){
    ros::NodeHandle localHandle("~");
    ros::ServiceServer id_service = localHandle.advertiseService("get_ROS_ID", &rdt::RosTable::GetID, &(this->commandsRos));
    ros::ServiceServer req_service = localHandle.advertiseService("req_ROS_Data", &rdt::RosTable::GetData, &(this->commandsRos));

    printf("starting:\n");
    const int delta = 1000;
    int curr = SDL_GetTicks();
    int threshold = curr + delta;
    while(ros::ok()){
        SDL_Event event;
        curr = SDL_GetTicks();
        /* Grab all the events off the queue. */
        while( SDL_PollEvent( &event ) ) {
            switch( event.type ) {
            case SDL_KEYDOWN:
                /* Handle key presses. */
                handle_key_down(&event.key.keysym);
                send((this->*SendFunc)());
                threshold = curr+delta;
                break;
            case SDL_KEYUP:
                /* Handle key releases. */
                handle_key_up(&event.key.keysym);
                send((this->*SendFunc)());
                threshold = curr+delta;
                break;
            case SDL_QUIT:
                return;
            }
        }
        if(threshold<curr){
            threshold = curr+delta;
            std::vector<char> header;
            header.resize(sizeof(curr) + commandsRos.nextSend.size());
            for(int i=0;i<sizeof(curr);++i){
                header[i] = (char)(curr & 0xFF);
                curr = curr >> 8;
            }
            for(int i = 0;i< commandsRos.nextSend.size();++i){
                header[sizeof(curr) + i] = commandsRos.nextSend[i];
            }
            send(header);
        }
        ros::spinOnce();
    }
}

std::vector<char> rdt::Manager::sendSend(){
    if(commands.hasUpdate(keyCompressed ^ keyCompressedPrev)){
        std::vector<char> header;
        int tick = SDL_GetTicks();
        int shift = sizeof(tick);
        header.resize(shift + commandsRos.nextSend.size());
        for(int i=0;i<sizeof(tick);++i){
            header[i] = (char)(tick & 0xFF);
            tick = tick >> 8;
        }
        for(int i = 0;i< commandsRos.nextSend.size();++i){
            header[shift+i] = commandsRos.nextSend[i];
            //printf("%d ",(int)commandsRos.nextSend[i]);
        }
        commands.encode(keyCompressed, keyCompressedPrev, header);
        keyCompressedPrev = keyCompressed;
        //printf("done\n");
        return header;
    }
    return {};
}

void rdt::Manager::sendRecv(const std::vector<char>& data){
    if(data.size()==0){return;}
    //printf("Start Recieve\n");
    int tmp1 = (int)data[0];
    commandsRos.decode(tmp1, std::string(data.begin()+1,data.end()));
    //printf("tmp1 %d\n", tmp1);
    int tmp2 = tmp1 >> 3;
    //printf("tmp2 %d\n", tmp2);
    tmp1 -= tmp2 << 3;
    //printf("tmp1 %d\n", tmp1);    

    commandsRos.nextSend[tmp2] &= (char)~(1<<tmp1);
    
    /*for(int i=0;i<data.size();++i){
        printf("%d ", (int)data[i]);
    }
    printf("\n");*/
    //commandsRos()

}

void rdt::Manager::recvLoop(){
    ros::NodeHandle localHandle("~");
    ros::ServiceServer id_service = localHandle.advertiseService("get_ROS_ID", &rdt::RosTable::GetID, &(this->commandsRos));
    ros::ServiceServer set_service = localHandle.advertiseService("set_ROS_Data", &rdt::RosTable::SetData, &(this->commandsRos));
    while(ros::ok()){
        ros::spinOnce();
    }
}

std::vector<char> rdt::Manager::recvSend(){
    return {};
}

void rdt::Manager::recvRecv(const std::vector<char>& data){
    int sC = commandsRos.nextSend.size();
    int sI = sizeof(int);
    if(data.size() < sC + sI){return;}
    for(int i=0;i<data.size();++i){
        printf("%d ",(int)data[i]);
    }
    printf("\n");
    commands.parse(data, sI + sC);

    for(int i = 0; i < sC; ++i){
        char c = data[sI + i];
        if(c != 0){
            for(int j=0;j<8;++j){
                //printf("At %d, %d\n",(i*8+j),((c & (1<<j))));
                if(((c & (1<<j))) != 0){
                    //printf("got %d\n",(i*8+j));
                    send(commandsRos.valAt(i*8+j));
                }
            }
        }
    }

}

rdt::Manager::Manager(int argc, char** argv, const char* title, int width, int height):width(width),height(height),headerSize(0){
    ros::init(argc, argv, "framework_node");
    handler = new ros::NodeHandle("");
    sender = true;
    std::string s;
    if(handler->hasParam("type")){
        handler->getParam("type",s);
        if (s == "sender"){}
        else if (s == "reciever"){ sender = false; }
        else{ ROS_INFO("Wrong value parameter _type:=<sender/reciever>. Defaulting to sender\n"); }
    }
    else{ ROS_INFO("Expecting parameter _type:=<sender/reciever>. Defaulting to sender\n"); }

    keyCompressed = 0U;
    keyCompressedPrev = 0U;
    
    sendHandler = handler->advertise<std_msgs::String>("SendBuffer", 1);
    recvHandler = handler->subscribe("RecvBuffer", 1, &rdt::Manager::RecieveCallback, this);

    if (sender){
        gl_setup(title, width, height);

        LoopFunc = &rdt::Manager::sendLoop;
        SendFunc = &rdt::Manager::sendSend;
        RecvFunc = &rdt::Manager::sendRecv;
    }
    else{
        LoopFunc = &rdt::Manager::recvLoop;
        SendFunc = &rdt::Manager::recvSend;
        RecvFunc = &rdt::Manager::recvRecv;
    }
}

rdt::Manager::~Manager(){
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void rdt::Manager::loop(){
    commands.setup(sender);
    commandsRos.setup(sender);

    (this->*LoopFunc)();
}
/*
void rdt::Manager::send(){
    std_msgs::String msg;
    std::vector<char> data = (this->*SendFunc)();
    if(data.size()==0){ return;}
    msg.data = std::string(data.begin(),data.end());
    sendHandler.publish(msg);
}*/
void rdt::Manager::send(const std::vector<char>& data){
    send(std::string(data.begin(),data.end()));
}
void rdt::Manager::send(const std::string& data){
    std_msgs::String msg;
    msg.data = data;
    sendHandler.publish(msg);
}

void rdt::Manager::recv(const std::vector<char>& data){
    (this->*RecvFunc)(data);
}

void rdt::Manager::RecieveCallback(const std_msgs::String::ConstPtr& msg){
    recv(std::vector<char>(msg->data.begin(), msg->data.end()));
}

bool SendImmediate(framework::IntStr::Request& req, framework::IntStr::Response& res){
    
}
