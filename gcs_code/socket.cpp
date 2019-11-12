// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <bits/stdc++.h> 
#include <string.h>
#include <net/if.h> 
#define PORT 8080
class Server{
private:
    int server_fd, new_socket, valread;

    std::vector<unsigned char> data;

    struct sockaddr_in myaddress;
    struct sockaddr_in address;

    int opt = 1;
    int PORT_NUM = 42069;
    int addrlen = sizeof(address);

    char str[INET_ADDRSTRLEN];
    char *ip = "127.0.0.1";
    char buffer[1024] = {0};
    char *hello = "Hello from server";
public:
    Server(int argc, char const *argv[]){
	
	if (argc > 3) {
	fprintf(stderr, "Too much Input\n");
	exit(1);
	}
	else if (argc == 3) {
	PORT_NUM = atoi(argv[1]);
	ip = (char*) argv[2];
	}
	else if (argc == 2) {
	PORT_NUM = atoi(argv[1]);
	}
        
        
        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }
        
        // Forcefully attaching socket to the port 8080
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,&opt, sizeof(opt)))
        {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }
        myaddress.sin_family = AF_INET;
	if(ip==NULL){
        	myaddress.sin_addr.s_addr = INADDR_ANY;
	}
	else{
		inet_pton(AF_INET,ip, &myaddress.sin_addr.s_addr);
	}
        myaddress.sin_port = htons( PORT_NUM );
        
        // Forcefully attaching socket to the port 8080
        if (bind(server_fd, (struct sockaddr *)&myaddress,
                                    sizeof(myaddress))<0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
	//Listen to the port
        if (listen(server_fd, 3) < 0)
        {
            perror("listen");
            exit(EXIT_FAILURE);
        }
	//If correctly connected
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                        (socklen_t*)&addrlen))<0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }
        printf("The GCC is on IP:%s Port:%d \n",ip,ntohs(myaddress.sin_port));
	printf("The Robot is on IP:%s Port:%d \n",inet_ntoa(address.sin_addr),ntohs(address.sin_port));
        valread = read( new_socket , buffer, 1024);
        printf("%s\n",buffer );
        send(new_socket , hello , strlen(hello) , 0 );
        //recv();
        printf("Hello message sent\n");
    }
    void initSocket(){
        if(this){
            
            Server* newServer = new Server( *this);
            this->destroy();

            
        }
    }
    void destroy(){
        close(1);
    }
    void changeTarget(){
        
    }
    
};

int main(int argc, char const *argv[])
{

    Server(argc,argv);
    
}
