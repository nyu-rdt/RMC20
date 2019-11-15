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
#include <pthread.h>
#define PORT 8080
#define NUM_THREADS 5
namespace rdt{
//listenfd is what we listen to
template <typename variable>
void *Listen(void *threadid,listenfd,void(variable::* nameoffunctionpointer)(vector<unsigned char>),void* charlesclass) {
	long tid = (long)thread;
	unsigned char buffer[1024];
	cout << "Thread with id : " << tid << endl;
	while(1){
	if(listenfd>0){
		int size = recv(listenfd,buffer,size(buffer)); //get buffer thing
		vector<unsigned char> data;
		for(i=0;i<size;i++){
			data.push_back(buffer[i]);	
		}
		((variable*)charlesclass)->*nameoffunctionpointer)(data);
	}
	cout << "Thread with id : " << tid << "  ...exiting " << endl;
}
//writefd we write to
//158
template <typename variable>
void *write(void *thread,writefd,vector<unsigned char>(variable::* nameoffunctionpointer)(void),void* charlesclass){
	std::vector<char> data;
	long tid = (long)thread;
	cout << "Thread with id : " << tid << endl;
	while(1){
		data = (((variable*)charlesclass->*nameoffunctionpointer)(void);
		if(data.size()){
			send(writefd , &data[0] , data.size() , 0 );
		}
	}
	cout << "Thread with id : " << tid << "  ...exiting " << endl;
}

class Server{
private:
    int server_fd, new_socket, valread;

    struct sockaddr_in myaddress; //STRUCT TO HOLD PACKET INFO THIS SERVER
    struct sockaddr_in address; //STRUCT TO HOLD PACKET INFO THIS ROBOT

    int opt = 1;
    int PORT_NUM = 42069; // Dummy PORT holder
    int addrlen = sizeof(address);

    char str[INET_ADDRSTRLEN];
    char *ip = "127.0.0.1";  // Dummy IP holder
    char buffer[1024] = {0};
    char *hello = "Hello from server";
public:
    Socket(){
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
        myaddress.sin_family = AF_INET; //TYPE OF PROTCOL
	if(ip==NULL){
        	myaddress.sin_addr.s_addr = INADDR_ANY; // GET ANY FREE IP
	}
	else{
		inet_pton(AF_INET,ip, &myaddress.sin_addr.s_addr); //SET INOUT IP
	}
        myaddress.sin_port = htons( PORT_NUM ); //SET PORT
        
        // Forcefully attaching socket to the port 8080
        if (bind(server_fd, (struct sockaddr *)&myaddress,
                                    sizeof(myaddress))<0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
	// Listen to the port
        if (listen(server_fd, 3) < 0)
        {
            perror("listen");
            exit(EXIT_FAILURE);
        }
	// If correctly connected
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                        (socklen_t*)&addrlen))<0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }
        printf("The GCC is on IP:%s Port:%d \n",ip,ntohs(myaddress.sin_port));
	printf("The Robot is on IP:%s Port:%d \n",inet_ntoa(address.sin_addr),ntohs(address.sin_port));
    }
    threads(){
	int rc;
	int i;
	pthread_t threads[NUM_THREADS];
	pthread_attr_t attr;
	void *status;


	// Initialize and set thread joinable
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);


	// Initialize and set thread joinable
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	i = 0;
	cout << "main() : creating thread, " << i << endl;
	rc = pthread_create(&threads[i], &attr, Listen, (void*) i,new_socket[0] );
	if (rc) {
	 cout << "Error:unable to create thread," << rc << endl;
	 exit(-1);
	}
	i=2;
	cout << "main() : creating thread, " << i << endl;
	rc = pthread_create(&threads[i], &attr, Write, (void*) i,new_socket[1], //charles method,charles class instance);
	if (rc) {
	 cout << "Error:unable to create thread," << rc << endl;
	 exit(-1);
	}

	// free attribute and wait for the other threads
	pthread_attr_destroy(&attr);
	for( i = 0; i < NUM_THREADS; i++ ) {
	rc = pthread_join(threads[i], &status);
		if (rc) {
		 cout << "Error:unable to join," << rc << endl;
		 exit(-1);
		}
	cout << "Main: completed thread id :" << i ;
	cout << "  exiting with status :" << status << endl;
	}

        valread = read( new_socket , buffer, 1024);
        printf("%s\n",buffer );
        send(new_socket , hello , strlen(hello) , 0 );
        //recv();
        printf("Hello message sent\n");
	cout << "Main: program exiting." << endl;
	pthread_exit(NULL);
    }
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
        
        socket();
        threads();
    }
    
};
}
int main(int argc, char const *argv[])
{

    rdt::Server(argc,argv);
    
}

