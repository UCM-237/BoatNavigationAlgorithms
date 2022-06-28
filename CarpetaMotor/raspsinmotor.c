#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringSerial.h>
#include <string.h>

#define ENC_ERROR 0 //valores de error como 0

int hall1_pin = 0; //pin lectura hall 1, cable azul
int hall2_pin = 1; //pin lectura hall 2, cable amarillo

int pulsador_pin = 2; //pin lectura pulsador de posición cero

int in1 = 3; //pines control giro
int in2 = 4;


float t_estabilizacion = 5000; //tiempo estabilizacion sonda en ms
float diam_bobina=0.04; //diámetro del carrete en metros
int confi_mala = 50; //confianza a partir de la cual no se realiza medida

int inc, previoA, previoB, actualA,actualB, ind,pulsos, vueltas, pulsador, d,fd,en,data_av,p;
int tiempo,checksum, checksumppzz, profundidad,d_sonar,c_sonar;
long int latitud,longitud,altitud;
char hex_tiempo[4],hex_profundidad[8],hex_checksum[2];
//char hex_long[5], hex_lat[5],hex_alt[4],hex_d_sonar[4], hex_c_sonar[1];

time_t t;
struct tm *tm;
char name_telemetria[100];
FILE *telemetria;


#define PPZ_START_BYTE 0x50 // "P"
#define COM_START_BYTE 0x52 // "R"
#define PPZ_SONAR_BYTE 0x53 // "S"
#define PPZ_TELEMETRY_BYTE 0x54 // "T"
#define PPZ_MEASURE_BYTE 0x4D // "M"
#define COM_FINAL_BYTE 0x15 //"F"
#define COM_ANSWER_BYTE 0x4F //"O"
#define COM_NO_MEASURE_BYTE 0x78 //"N"
#define POSITIVO 0x00 
#define NEGATIVO 0x01


//función para pasar enteros a hexadecimal
void itoh(int value, unsigned char* str, int nbytes){
	double nmax;
	int nb;
	nmax=pow(2,(nbytes-1)*8)-1;
	nb=nbytes;
	if (abs(value)>nmax) return;
	if (value<0){
		str[0]=1;
		value=-value;
	}
	else str[0]=0;
	int i;
	for (i=1; i<nb; i++){
		str[i]=(value & (0xff << (nb-1-i)*8)) >> (8*(nb-1-i));
	}
	return ;
}

//función para pasar bytes a enteros
unsigned int serial_byteToint(char bytes[], int length){
	unsigned int num = 0;
	int i=length-1;
	while(i>=0){
		num=num|bytes[i]<<(8*(i));
		i--;
	}
	return num;
}

//función para comprobar checksum
 int compare_checksum(char message[], int check, int l_message){
	int i = 0;
	int suma=0;
	while(i<l_message-2){
		suma=suma+(int)message[i];
		i++;
	}
	if(suma==check){
		return 1;
	}
	if(suma!=check){
		return 0;
	}
}

//función para ver el signo en los bytes con signo
int calculate_signo(char signo){
	if (signo==POSITIVO){
		return 1;
	}
	if (signo==NEGATIVO){
		return -1;
	}
}


//hilo contador del tiempo
 PI_THREAD (contador_t){
	 tiempo = 0;
	 for(;;){
		 delay(1000);
		 tiempo++;
	 }
 }

 
//hilo de comunicación con papparazzi
PI_THREAD(comunicacion_ppzz){
	for(;;){
		printf("\n\n");
		data_av = serialDataAvail(fd);
		p=0;
		printf("Número bytes: %i\n", data_av);
		if(data_av>0){
			char datappzz[data_av];
			while(p<data_av){
				datappzz[p] = serialGetchar(fd);
				p++;
				}
			//printf("%s\n", datappzz);
			if(datappzz[0]!=PPZ_START_BYTE){
				printf("NO EMPIEZA POR P\n");
				delay(1000);
				continue;
			}
			if(strchr(datappzz,PPZ_TELEMETRY_BYTE)!=NULL){ //Mensaje telemetria
				
				if(data_av!=25){
					printf("NÚMERO BYTES INCORRECTO\n");
					delay(1000);
					continue;
				}
				
				char hex_checksumppzz[2] = {datappzz[24],datappzz[23]};
				checksumppzz = serial_byteToint(hex_checksumppzz,2);
				
				if(compare_checksum(datappzz, checksumppzz, data_av)==0){
					printf("Checksum erróneo.\n");
					delay(1000);
					continue;
				}
				
				printf("TELEMETRIA\n");
				en=0;
				
				char sign_long = datappzz[4];
				char hex_long[4]={datappzz[8],datappzz[7],datappzz[6],datappzz[5]};
				longitud = calculate_signo(sign_long)*serial_byteToint(hex_long,4);
				
				char sign_lat = datappzz[9];
				char hex_lat[4]={datappzz[13],datappzz[12],datappzz[11],datappzz[10]};
				latitud = calculate_signo(sign_lat)*serial_byteToint(hex_lat,4);
				
				char sign_alt = datappzz[14];
				char hex_alt[4]={datappzz[17],datappzz[16],datappzz[15]};
				altitud = calculate_signo(sign_alt)*serial_byteToint(hex_alt,3);
				
				char hex_d_sonar[5]={datappzz[21],datappzz[20],datappzz[19],datappzz[18]};
				d_sonar = serial_byteToint(hex_d_sonar,4);
				
				char hex_c_sonar = datappzz[22];
				c_sonar = (int)hex_c_sonar;
				
				
				printf("Longitud: %li, Latitud: %li, Altitud: %li, D_Sonar: %li, C_Sonar: %li, Checksum: %i\n",
						longitud, latitud, altitud, d_sonar, c_sonar,checksumppzz);
						
				telemetria = fopen(name_telemetria,"a"); //se abre el archivo añadir GPS
				fprintf(telemetria, "%li %li %li %li %li %li\n",tiempo,longitud,latitud,altitud,d_sonar,c_sonar);
				fclose(telemetria);
			}
			if(strchr(datappzz,PPZ_MEASURE_BYTE)!=NULL){ //Mensaje inicio de medida
				
				if(data_av!=25){
					printf("NÚMERO BYTES INCORRECTO\n");
					delay(1000);
					continue;
				}
				
				char hex_checksumppzz[2] = {datappzz[24],datappzz[23]};
				checksumppzz = serial_byteToint(hex_checksumppzz,2);
				
				if(compare_checksum(datappzz, checksumppzz, data_av)==0){
					printf("Checksum erróneo.\n");
					delay(1000);
					continue;
				}
				
				printf("MEDIDA\n");
				en=1;
				
				char sign_long = datappzz[4];
				char hex_long[4]={datappzz[8],datappzz[7],datappzz[6],datappzz[5]};
				longitud = calculate_signo(sign_long)*serial_byteToint(hex_long,4);
				
				char sign_lat = datappzz[9];
				char hex_lat[4]={datappzz[13],datappzz[12],datappzz[11],datappzz[10]};
				latitud = calculate_signo(sign_lat)*serial_byteToint(hex_lat,4);
				
				char sign_alt = datappzz[14];
				char hex_alt[4]={datappzz[17],datappzz[16],datappzz[15]};
				altitud = calculate_signo(sign_alt)*serial_byteToint(hex_alt,3);
				
				char hex_d_sonar[5]={datappzz[21],datappzz[20],datappzz[19],datappzz[18]};
				d_sonar = serial_byteToint(hex_d_sonar,4);
				
				char hex_c_sonar = datappzz[22];
				c_sonar = (int)hex_c_sonar;
				
				printf("Longitud: %li, Latitud: %li, Altitud: %li, D_Sonar: %li, C_Sonar: %li, Checksum: %i\n",
						longitud, latitud, altitud, d_sonar, c_sonar,checksumppzz);
				telemetria = fopen(name_telemetria,"a"); //se abre el archivo añadir GPS
				fprintf(telemetria, "%li %li %li %li %li %li\n",tiempo,longitud,latitud,altitud,d_sonar,c_sonar);
				fclose(telemetria);
				
				//MENSAJE RESPUESTA A SOLICITUD DE MEDIDA
				checksum = COM_START_BYTE+COM_ANSWER_BYTE+tiempo;
				itoh(tiempo, hex_tiempo, 2);
				itoh(checksum, hex_checksum, 2);
				serialPutchar(fd,COM_START_BYTE);
				serialPutchar(fd,COM_ANSWER_BYTE);
				serialPutchar(fd,hex_tiempo[0]);
				serialPutchar(fd,hex_tiempo[1]);
				serialPutchar(fd,hex_checksum[0]);
				serialPutchar(fd,hex_checksum[1]);
				
			}
			if(strchr(datappzz,PPZ_SONAR_BYTE)!=NULL){
				
				if(data_av!=6){
					printf("NÚMERO BYTES INCORRECTO\n");
					delay(1000);
					continue;
				}
				
				char hex_checksumppzz[2] = {datappzz[5],datappzz[4]};
				checksumppzz = serial_byteToint(hex_checksumppzz,2);
				
				if(compare_checksum(datappzz, checksumppzz, data_av)==0){
					printf("Checksum erróneo.\n");
					delay(1000);
					continue;
				}
				
				en=0;
				
				//MENSAJE RESPUESTA A SOLICITUD DE PROFUNDIDAD
				checksum = COM_START_BYTE+PPZ_START_BYTE+tiempo+profundidad;
				itoh(tiempo, hex_tiempo, 2);
				itoh(profundidad, hex_profundidad, 4);
				itoh(checksum, hex_checksum, 2);
				serialPutchar(fd,COM_START_BYTE);
				serialPutchar(fd,PPZ_START_BYTE);
				serialPutchar(fd,hex_tiempo[0]);
				serialPutchar(fd,hex_tiempo[1]);
				serialPutchar(fd,hex_profundidad[0]);
				serialPutchar(fd,hex_profundidad[1]);
				serialPutchar(fd,hex_checksum[0]);
				serialPutchar(fd,hex_checksum[1]);
			}
		}
		delay(1000);
	} 	 
}
 
int i,j,v;
float p_d;
 
//función de rutina de bajada
//bajar hasta d(m) y subir parando en intervalos
 void rutina(float d_sonar, int intervalo){ // intervalo==0 => 0.5m  intervalo==1 => 1m
	 j=0;
	 d_sonar=d_sonar/1000;
	 //primero comprobar que es posible bajar
	 if(1==0){
		 d=0;
		 printf("No es posible medida.\n");
		 
		 //MENSAJE DE NO MEDIDA
		 checksum = COM_START_BYTE+COM_NO_MEASURE_BYTE+tiempo;
		 itoh(tiempo, hex_tiempo, 2);
		 itoh(checksum, hex_checksum, 2);
		 serialPutchar(fd,COM_START_BYTE);
		 serialPutchar(fd,COM_NO_MEASURE_BYTE);
		 serialPutchar(fd,hex_tiempo[0]);
		 serialPutchar(fd,hex_tiempo[1]);
		 serialPutchar(fd,hex_checksum[0]);
		 serialPutchar(fd,hex_checksum[1]);
		
	 }
	 else{
		 d=floor(d_sonar-0.5); //cálculo profundidad de bajada
		 
		 if(intervalo==0){ //bajada cada 0.5m
			 v = 2;
			 p_d = 0.5;
		 }
		 if(intervalo==1){ //bajada cada 1m
			 v = 1;
			 p_d = 1;
		 }

		 printf("Comienzo rutina.\n");
		 
		delay(2000);
		printf("Sensor en posición cero.\n"); 
		//RM tiempo(2bytes) profundidad en mm (2 bytes) checksum 
		checksum = COM_START_BYTE+PPZ_MEASURE_BYTE+tiempo+profundidad;
		itoh(tiempo, hex_tiempo, 2);
		itoh(profundidad, hex_profundidad, 2);
		itoh(checksum, hex_checksum, 2);
		
		serialPutchar(fd,COM_START_BYTE);
		serialPutchar(fd,PPZ_MEASURE_BYTE);
		serialPutchar(fd,hex_tiempo[0]);
		serialPutchar(fd,hex_tiempo[1]);
		serialPutchar(fd,hex_profundidad[0]);
		serialPutchar(fd,hex_profundidad[1]);
		serialPutchar(fd,hex_checksum[0]);
		serialPutchar(fd,hex_checksum[1]);
	
		delay(1000);
		
		i=0;
		
		while(i<d*v){ // va bajando cada distancia dada
			delay(1000);
			profundidad = (i+1)*p_d*1000;
			printf("Ha llegado a %i m.\n",profundidad); //pone en pantalla a qué profundidad llegó
			//MENSAJE
			//RM tiempo(2bytes) profundidad en mm (2 bytes) checksum 
			checksum = COM_START_BYTE+PPZ_MEASURE_BYTE+tiempo+profundidad;
			itoh(tiempo, hex_tiempo, 2);
			itoh(profundidad, hex_profundidad, 2);
			itoh(checksum, hex_checksum, 2);
			serialPutchar(fd,COM_START_BYTE);
			serialPutchar(fd,PPZ_MEASURE_BYTE);
			serialPutchar(fd,hex_tiempo[0]);
			serialPutchar(fd,hex_tiempo[1]);
			serialPutchar(fd,hex_profundidad[0]);
			serialPutchar(fd,hex_profundidad[1]);
			serialPutchar(fd,hex_checksum[0]);
			serialPutchar(fd,hex_checksum[1]);
			
			delay(t_estabilizacion); //espera tiempo necesario para estabilizar
			i++;
		}
		i=i-2;
		while(i>-1){ // va subiendo cada distancia dada
			delay(1000);
			profundidad = (i+1)*p_d*1000;
			printf("Ha llegado a %i m.\n",profundidad); //pone en pantalla a qué profundidad llegó
			//MENSAJE
			//RM tiempo(2bytes) profundidad en mm (2 bytes) checksum
			checksum = checksum = COM_START_BYTE+PPZ_MEASURE_BYTE+tiempo+profundidad;
			itoh(tiempo, hex_tiempo, 2);
			itoh(profundidad, hex_profundidad, 2);
			itoh(checksum, hex_checksum, 2);
			serialPutchar(fd,COM_START_BYTE);
			serialPutchar(fd,PPZ_MEASURE_BYTE);
			serialPutchar(fd,hex_tiempo[0]);
			serialPutchar(fd,hex_tiempo[1]);
			serialPutchar(fd,hex_profundidad[0]);
			serialPutchar(fd,hex_profundidad[1]);
			serialPutchar(fd,hex_checksum[0]);
			serialPutchar(fd,hex_checksum[1]);
			
			delay(t_estabilizacion); //espera tiempo necesario para estabilizar
			 
			i--;
		 }
		 //vuelta a cero
		 printf("Vuelta a cero.\n");

		 delay(2000); //espera 2s
		 printf("Sensor en posición cero.\n");
		 //RM tiempo(2bytes) profundidad en mm (2 bytes) checksum
		 checksum = COM_START_BYTE+PPZ_MEASURE_BYTE+tiempo+profundidad;
		 itoh(tiempo, hex_tiempo, 2);
		 itoh(profundidad, hex_profundidad, 2);
		 itoh(checksum, hex_checksum, 2);
		 serialPutchar(fd,COM_START_BYTE);
		 serialPutchar(fd,PPZ_MEASURE_BYTE);
		 serialPutchar(fd,hex_tiempo[0]);
		 serialPutchar(fd,hex_tiempo[1]);
		 serialPutchar(fd,hex_profundidad[0]);
		 serialPutchar(fd,hex_profundidad[1]);
		 serialPutchar(fd,hex_checksum[0]);
		 serialPutchar(fd,hex_checksum[1]);
		
		 
		 //Recogida de datos
		 system("./prueba");
		 
		 
		 //Cambio de nombre al archivo de datos y añadir GPS
		 char sensor_arch_name[100];
		 
		 t=time(NULL);
		 tm=localtime(&t);
		 strftime(sensor_arch_name,100,"Sensor-%X-%d-%m-%y.txt",tm); //se crea el nombre del archivo según fecha y hora
		 
		 rename("sensordata.txt", sensor_arch_name); //se cambia al nombre creado
		 
		 
		 FILE *archivo = fopen(sensor_arch_name,"a+"); //se abre el archivo para añadir GPS al sensor
		 fprintf(archivo, "Coord. GPS:\n  Latitud &i\n Longitud %i\n Altitud %i\n",longitud,latitud,altitud);
		 fclose(archivo);
		 
		 printf("Fin de rutina.\n");
		 //RM tiempo(2bytes) profundidad en mm (2 bytes) checksum
		 checksum = COM_START_BYTE+COM_FINAL_BYTE+tiempo+profundidad;
		 itoh(tiempo, hex_tiempo, 2);
		 itoh(profundidad, hex_profundidad, 4);
		 itoh(checksum, hex_checksum, 2);
		 serialPutchar(fd,COM_START_BYTE);
		 serialPutchar(fd,COM_FINAL_BYTE);
		 serialPutchar(fd,hex_tiempo[0]);
		 serialPutchar(fd,hex_tiempo[1]);
		 serialPutchar(fd,hex_profundidad[0]);
		 serialPutchar(fd,hex_profundidad[1]);
		 serialPutchar(fd,hex_checksum[0]);
		 serialPutchar(fd,hex_checksum[1]);
	 }
	 
	 
	 en=0;
 }
 
 int main(){
	 wiringPiSetup();
	 pinMode(hall1_pin,INPUT);
	 pinMode(hall2_pin,INPUT);
	 pinMode(pulsador_pin,INPUT);
	 pinMode(in1, OUTPUT);
	 pinMode(in2,OUTPUT);
	 
	 fd = serialOpen("/dev/serial0", 9600); //Puerto serie
	 
	 
	 t=time(NULL);
	 tm=localtime(&t);
	 //se crea el nombre del archivo de telemetria según fecha y hora
	 strftime(name_telemetria,100,"Telemetria-%X-%d-%m-%y.txt",tm); 
	 
	 telemetria = fopen(name_telemetria,"w+"); //se abre el archivo escribir GPS
	 fprintf(telemetria, "TIME LATITUD LONGITUD ALTITUD PROFUNDIDAD CONFIANZA\n");
	 fclose(telemetria);
	 
	 piThreadCreate(contador_t);
	 piThreadCreate(comunicacion_ppzz);
	 
	 profundidad=0;
	 
	 delay(500);
	 
	 for(;;){
		 printf("%i\n", en);
		 if(en==1){
			 rutina(d_sonar,0);
		 }
		 if(en==0){
			 printf("Esperando disparo\n");
		 }
		 delay(1000);
	 }
	 
	 return 0;
	 
 }
