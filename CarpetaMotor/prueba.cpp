
#include "sonda.cpp"
#include <time.h>
#include <iostream>
#include  <stdlib.h>

using namespace std;

sonda sonda1;
int main(){

    sonda1.startSendThread();
    sonda1.startRecvThread();

    sonda1.Time();
    
    time_t rawtime;
    time(&rawtime);
    
    struct tm*tlocal = localtime(&rawtime);

    usleep(100);
        //sonda1.TaskConfiguration(1,"punctual",100,1);
      /* usleep(100);
        sonda1.Time();
        usleep(100);*/
        
        char aa[5];
        char mm[3];
        char dd[3];
        char hh[10];
        
        string nombre("prueba2");
        strftime(aa,5,"%y",tlocal);
        strftime(mm,3,"%m",tlocal);
        strftime(dd,3,"%d",tlocal);
        strftime(hh,10,"X",tlocal);
        
        
        //nombre += hh;
        
        sonda1.DataCsv("prueba2",aa,mm,dd);
        cout<<"datosGuardados"<<endl;
        //while(true){

        //}

    

    return 0;

}
