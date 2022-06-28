#ifndef SONDA_HH
#define SONDA_HH
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include <sstream>
#include <vector>

const unsigned int MAXSIZE = 512;
class sonda{
    private:
    
    int socketSonda;
    FILE *log;				// File log
	pthread_t _send,_recv;		// thread

    std::string sendBuf;
    char recvBuf[MAXSIZE];
    std::string newLine="\n";
    bool Save=false;
    public:
    //thread functions
    void startSendThread();//start thread
    void startRecvThread();//start thread

    void sendFunc();//send function for thread
    void recvFunc();//recv function for thread
    static void *sendThread(void *This);
    static void *recvThread(void *This);

    //communication functions
    void ReciveCommunication();
    void DataCsv(std::string name,std::string year,std::string month,std::string day);
    void TaskConfiguration(int numTask,std::string type,int period,int record);
    void CreateTask();
    void taskExecute(int n);
    void Time();
    void saveData();
    //constructor and desctructor
    sonda();
    ~sonda();
};

#endif
