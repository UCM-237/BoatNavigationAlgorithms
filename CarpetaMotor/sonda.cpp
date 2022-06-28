#include "sonda.hh"
#include <iostream>

sonda::sonda(){//constructor

    // pthread initialization
	_send = pthread_self(); // get pthread ID
    _recv = pthread_self(); // get pthread ID
	//pthread_setschedprio(_thread, SCHED_FIFO); // setting priority
    log = fopen("sensordata.txt", "w");

    const char *ip = "8.8.8.8";//ip of server
	unsigned short int port = atoi("800");//port of server

	socketSonda = socket(AF_INET,SOCK_STREAM,0);
	if(socketSonda == -1){
		perror("socket");
		//return -1;
	}
	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr(ip);
	socklen_t addrlen = sizeof(addr);

	int ret = connect(socketSonda,(const struct sockaddr*)(&addr),addrlen);
	if(ret == -1){
		perror("connect");
		//return -1;
	}
    sendBuf.clear();//clean buf
}

void sonda::sendFunc(){
    
     while(1){
			/*int numtask=1;
			std::string type="punctual";
			int period =1000;
			std::string config="task config";
    		std::string space = " ";
    		std::string hola=config+space +  std::to_string(numtask)+space + type+space + std::to_string(period);*/
            
			
            if(sendBuf.size()>0){
                //fgets(buf,sizeof(buf),stdin);
                if(send(socketSonda,sendBuf.c_str(),sendBuf.size(),0)<=0){
                    break;	
                }
                sendBuf.clear();
            }
		}

    
}
void sonda::recvFunc(){

    while(1){
			//all recived must be stored in a file.
			std::string r;
			if(recv(socketSonda,&recvBuf,MAXSIZE,0)<=0){
				break;
			}
			time_t timep;
			time(&timep);//timeStamp
            if(Save){
                fwrite (recvBuf , sizeof(char), sizeof(recvBuf), log);
            }
            else{
                //std::cout<<ctime(&timep)<<std::endl;//
                std::cout<<recvBuf<<std::endl;
            }
			
		}

}
void sonda::startSendThread(){

    pthread_create(&_send,NULL,sendThread,this);

}
void sonda::startRecvThread(){
    pthread_create(&_recv,NULL,recvThread,this);
}

void *sonda::sendThread(void *This){

   ((sonda *)This)->sendFunc();
    return NULL;
}

void *sonda::recvThread(void *This){

   ((sonda *)This)->recvFunc();
    return NULL;
}
void sonda::CreateTask(){



}
void  sonda:: TaskConfiguration(int numTask,std::string type,int period,int record){
    //type is punctual continuius,profilre
    //period min 1 max 129600
    //record 1 true, 0 false
    std::string task="task";
    std::string config="config";
    std::string space = " ";
    std::string rec;
    if(record){
        rec="record-ON";
    }
    else{
        rec="record-OFF";
    }
     sendBuf=task+space+ std::to_string(numTask) + space+ config+ space+
     type +space+ std::to_string(period)+space+ rec + newLine;
    

}

void sonda::taskExecute(int n){//finctiomn for execute a task
    
}

void sonda::Time(){//function for set Time
    time_t timep;
	std::string stime="Time ";		
    sendBuf= stime + std::to_string(time(&timep)) +  newLine;
}
void sonda::DataCsv(std::string name,std::string year,std::string month,std::string day){//function for request data and store in a file
    std::string data="Data";
    std::string space=" ";
    std::string bar="_";
    std::string csv=".csv";
    Save=true;
    sendBuf=data + space + name + bar + year + 
            month +  day + csv +newLine;
}
void sonda::saveData(){
    fclose (log);
}
sonda::~sonda(){

    close(socketSonda);

}