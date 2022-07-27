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

int inc, previoA, previoB, actualA,actualB, ind,pulsos, vueltas, pulsador, d,fd,en,data_av,p,contador;
int tiempo,checksum, checksumppzz, profundidad,d_sonar,c_sonar,intervalo;
long int latitud,longitud,altitud;
char hex_tiempo[4],hex_profundidad[2],hex_checksum[2];
//char hex_long[5], hex_lat[5],hex_alt[4],hex_d_sonar[4], hex_c_sonar[1];

time_t t;
struct tm *tm;
int min_bajada,hora_bajada;
char name_telemetria[100];
char name_log[100];
char name_puntos_medida[100];
FILE *telemetria;
FILE *log_errores;
FILE *puntos_medida;

#define PPZ_START_BYTE 0x50 // "P"
#define COM_START_BYTE 0x52 // "R"
#define PPZ_SONAR_BYTE 0x53 // "S"
#define PPZ_TELEMETRY_BYTE 0x54 // "T"
#define PPZ_MEASURE_BYTE 0x4D // "M"
#define COM_FINAL_BYTE 0x46 //"F"
#define COM_ANSWER_BYTE 0x4F //"O"
#define COM_NO_MEASURE_BYTE 0x4E //"N"
#define COM_LOSS_BYTE 0x4C // "L"
#define COM_HALL_BYTE 0x48 //"H"
#define COM_VUELTA_BYTE 0x56 // "V"
#define POSITIVO 0x00 
#define NEGATIVO 0x01


//función para pasar enetros a hexadecimal
void itoh(int value, unsigned char* str, int nbytes){
	double nmax;
	int nb;
	nmax=pow(2,(nbytes)*8)-1;
	nb=nbytes;
	if (abs(value)>nmax) return;
	int i;
	for (i=0; i<nb; i++){
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

//matriz de pulsos de encoder según salidas hall
int M_inc[16]={0,-1,1,0,
	1,ENC_ERROR,ENC_ERROR,-1,
	-1,ENC_ERROR,ENC_ERROR,1,
	0,1,-1,0}; 
 //+1 pulso sentido horario
 //-1 pulso sentido antihorario
 //0 no hay movimiento
 
//función giro del motor
 void fgiro(int giro){
	 if(giro==1){ //bajar
		 digitalWrite(in1,0);
		 digitalWrite(in2,0);
		 delay(100);
		 digitalWrite(in1,1);
		 digitalWrite(in2,0);
	 }
	 if(giro==-1){ //subir
		 digitalWrite(in1,0);
		 digitalWrite(in2,0);
		 delay(100);
		 digitalWrite(in1,0);
		 digitalWrite(in2,1);
	 }
	 if(giro==0){ //parar
		 digitalWrite(in1,0);
		 digitalWrite(in2,0);
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
		 
//hilo medida de vueltas del motor
 PI_THREAD(contador_vueltas){
	 pulsador = digitalRead(pulsador_pin);
	 pulsos = 0; //inicializar a cero
	 vueltas = 0;
	 actualA = digitalRead(hall1_pin); //primera lectura
	 actualB = digitalRead(hall2_pin);
	 
	 for(;;){
		 previoA = actualA; //actualización
		 previoB = actualB;
		 
		 pulsador = digitalRead(pulsador_pin);
		 if(pulsador==1){ //sonda en posición cero
			 fgiro(0);
			 pulsos = 0; //inicializa a cero el encoder
			 vueltas = 0;
		 }
		 
		 actualA = digitalRead(hall1_pin); //lectura actual
		 actualB = digitalRead(hall2_pin);
		 ind = 8*previoA+4*actualA+2*previoB+actualB; //cálculo índice matriz
		 inc = M_inc[ind]; //incremento 
		 pulsos +=inc; //pulsos totales 
		 vueltas = pulsos/490; //cálculo vueltas
	 }
 }
 
//hilo de comunicación con papparazzi
PI_THREAD(comunicacion_ppzz){
	for(;;){
		data_av = serialDataAvail(fd);
		p=0;
		printf("Número bytes: %i\n", data_av);
		if(data_av>0){
			char datappzz[data_av];
			while(p<data_av){
				datappzz[p] = serialGetchar(fd);
				p++;
				}
			printf("%s\n", datappzz);
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
				fprintf(telemetria, "%li %c %li %li %li %li %li\n",tiempo,datappzz[1],longitud,latitud,altitud,d_sonar,c_sonar);
				fclose(telemetria);
			}
			if(strchr(datappzz,PPZ_MEASURE_BYTE)!=NULL){ //Mensaje inicio de medida
				
				if(data_av!=26){
					printf("NÚMERO BYTES INCORRECTO\n");
					delay(1000);
					continue;
				}
				
				char hex_checksumppzz[2] = {datappzz[25],datappzz[24]};
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
				
				char hex_intervalo = datappzz[23];
				intervalo = (int)intervalo;
				
				//printf("Longitud: %li, Latitud: %li, Altitud: %li, D_Sonar: %li, C_Sonar: %li, Checksum: %i\n",
						//longitud, latitud, altitud, d_sonar, c_sonar,checksumppzz);
				telemetria = fopen(name_telemetria,"a"); //se abre el archivo añadir GPS
				fprintf(telemetria, "%li %c %li %li %li %li %li\n",tiempo,datappzz[1],longitud,latitud,altitud,d_sonar,c_sonar);
				fclose(telemetria);
				
				puntos_medida = fopen(name_puntos_medida,"a"); //se abre el archivo añadir GPS
				fprintf(puntos_medida, "%li %c %li %li %li %li %li\n",tiempo,datappzz[1],longitud,latitud,altitud,d_sonar,c_sonar);
				fclose(puntos_medida);
				
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
	 printf("%li\n",d_sonar);
	 //primero comprobar que es posible bajar
	 if(d_sonar<=1||c_sonar<=confi_mala){
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
		 
		 log_errores = fopen(name_log,"a");
		 fprintf(log_errores, "%i %c : No hay suficiente profundidad para medir.\n",tiempo,COM_NO_MEASURE_BYTE);
		 fclose(log_errores);
		 
		 return;
		
	 }
	 
	 d=floor(d_sonar-0.5); //cálculo profundidad de bajada
	 
	 if(intervalo==0){ //bajada cada 0.5m
		 v = 2;
		 p_d = 0.5;
	 }
	 if(intervalo==1){ //bajada cada 1m
		 v = 1;
		 p_d = 1;
	 }
	 
	 //VECTOR VUELTAS
	 float vec_vueltas[d*v]; //vueltas para llegar a cada posición de 1m/0.5m
	 while(j<d*v){
		 vec_vueltas[j] = (j+1)*p_d/(M_PI*diam_bobina); //cada profundidad de 1m/0.5m entre el perimetro
		 j++;
	 }
	 printf("Comienzo rutina.\n");
	 
	 
	 
	//primero se va a colocar el sensor en posición cero
	digitalWrite(in1,0); //se garantiza que el motor esté parado
	digitalWrite(in2,0);
	contador = 0;
	while(pulsador==0){ //mientras no se pulse, sube
		digitalWrite(in1,0);
		digitalWrite(in2,1);
		contador++;
		delay(100);
		printf("%i\n",contador);
		if(contador>300){
			digitalWrite(in1,0);
			digitalWrite(in2,0); //se para el motor
			printf("Sonda perdida\n");
			//MENSAJE PERDIDA SONDA
			checksum = COM_START_BYTE+COM_LOSS_BYTE+tiempo;
			itoh(tiempo, hex_tiempo, 2);
			itoh(checksum, hex_checksum, 2);
			serialPutchar(fd,COM_START_BYTE);
			serialPutchar(fd,COM_LOSS_BYTE);
			serialPutchar(fd,hex_tiempo[0]);
			serialPutchar(fd,hex_tiempo[1]);
			serialPutchar(fd,hex_checksum[0]);
			serialPutchar(fd,hex_checksum[1]);
			
			log_errores = fopen(name_log,"a");
			fprintf(log_errores, "%i %c : La sonda no sube.\n",tiempo,COM_LOSS_BYTE);
			fclose(log_errores);
			
			en = 0;
			return;
		}
	} 

	digitalWrite(in1,0);
	digitalWrite(in2,0); //se para el motor
	
	printf("Sensor en posición cero.\n"); 
	
	t=time(NULL);
	tm=localtime(&t);
	
	char str_min_bajada[50];
	char str_hora_bajada[50];
	
	//Guardo la hora de bajada de la sonda
	strftime(str_min_bajada,50,"%M",tm);
	strftime(str_hora_bajada,50,"%H",tm);
	
	min_bajada = atoi(str_min_bajada);
	hora_bajada = atoi(str_hora_bajada);
	
	
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
	
	//COMIENZA BAJADA
	while(i<d*v){ // va bajando cada distancia dada
		contador = 0;
		while(vueltas<vec_vueltas[i]){ //baja 1m/0.5m
			digitalWrite(in1,1);
			digitalWrite(in2,0);
			contador++;
			delay(100);
			if(contador>300 & vueltas<vec_vueltas[i]){
				digitalWrite(in1,0); //para
				digitalWrite(in2,0);
				
				checksum = COM_START_BYTE+COM_HALL_BYTE+tiempo+profundidad;
				itoh(tiempo, hex_tiempo, 2);
				itoh(profundidad, hex_profundidad, 2);
				itoh(checksum, hex_checksum, 2);
				serialPutchar(fd,COM_START_BYTE);
				serialPutchar(fd,COM_HALL_BYTE);
				serialPutchar(fd,hex_tiempo[0]);
				serialPutchar(fd,hex_tiempo[1]);
				serialPutchar(fd,hex_profundidad[0]);
				serialPutchar(fd,hex_profundidad[1]);
				serialPutchar(fd,hex_checksum[0]);
				serialPutchar(fd,hex_checksum[1]);
				
				log_errores = fopen(name_log,"a");
				fprintf(log_errores, "%i %c : No hay lectura del encoder del motor.\n",tiempo,COM_HALL_BYTE);
				fclose(log_errores);
				
				printf("Pérdida motor\n");
				
				//Vamos a intentar subir la sonda por si solo está fallando el encoder
				contador = 0;
				while(pulsador==0){ //mientras no se pulse, sube
					digitalWrite(in1,0);
					digitalWrite(in2,1);
					contador++;
					delay(100);
					//printf("%i\n",contador);
					if(contador>300){
						digitalWrite(in1,0);
						digitalWrite(in2,0); //se para el motor
						printf("Sonda perdida\n");
						//MENSAJE PERDIDA SONDA
						checksum = COM_START_BYTE+COM_LOSS_BYTE+tiempo;
						itoh(tiempo, hex_tiempo, 2);
						itoh(checksum, hex_checksum, 2);
						serialPutchar(fd,COM_START_BYTE);
						serialPutchar(fd,COM_LOSS_BYTE);
						serialPutchar(fd,hex_tiempo[0]);
						serialPutchar(fd,hex_tiempo[1]);
						serialPutchar(fd,hex_checksum[0]);
						serialPutchar(fd,hex_checksum[1]);
						
						log_errores = fopen(name_log,"a");
						fprintf(log_errores, "%i %c : La sonda no sube.\n",tiempo,COM_LOSS_BYTE);
						fclose(log_errores);
						
						en = 0;
						return;
					}
				}
				//Si ha conseguido subir tras la pérdida de vueltas
				
				digitalWrite(in1,1); //se baja un poco para no mantener interruptor activado pero sin usar encoder porque no funciona
				digitalWrite(in2,0);
				delay(1000);
				digitalWrite(in1,0);
				digitalWrite(in2,0);
				
				//Mensaje de vuelta a casa segura
				checksum = COM_START_BYTE+COM_VUELTA_BYTE+tiempo;
				itoh(tiempo, hex_tiempo, 2);
				itoh(checksum, hex_checksum, 2);
				serialPutchar(fd,COM_START_BYTE);
				serialPutchar(fd,COM_VUELTA_BYTE);
				serialPutchar(fd,hex_tiempo[0]);
				serialPutchar(fd,hex_tiempo[1]);
				serialPutchar(fd,hex_checksum[0]);
				serialPutchar(fd,hex_checksum[1]);
				
				log_errores = fopen(name_log,"a");
				fprintf(log_errores, "%i %c : Vuelta segura a casa tras pérdida encoder motor.\n",tiempo,COM_VUELTA_BYTE);
				fclose(log_errores);
				
				en = 0;
				return;
			}
		}
		digitalWrite(in1,0); //para
		digitalWrite(in2,0);
		profundidad = (i+1)*p_d*1000;
		printf("Ha llegado a %i mm.\n",profundidad); //pone en pantalla a qué profundidad llegó
		//MENSAJE
		//RM tiempo(2bytes) profundidad en mm (2 bytes) checksum 
		checksum = COM_START_BYTE+PPZ_MEASURE_BYTE+tiempo+profundidad;
		printf("checksum: %i\n",checksum);
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
		printf("Mensaje: %X %X %X %X %X %X %X %X\n",COM_START_BYTE,PPZ_MEASURE_BYTE,hex_tiempo[0],hex_tiempo[1],
			hex_profundidad[0],hex_profundidad[1], hex_checksum[0], hex_checksum[1]);
		
		delay(t_estabilizacion); //espera tiempo necesario para estabilizar
		i++;
	}
	//COMIENZA SUBIDA
	i=i-2;
	while(i>-1){ // va subiendo cada distancia dada
		while(vueltas>vec_vueltas[i]){ //sube 1m
			digitalWrite(in1,0);
			digitalWrite(in2,1);
			if(contador>300 & vueltas<vec_vueltas[i]){ //Se pierde el lector Hall o el motor
				digitalWrite(in1,0); //para
				digitalWrite(in2,0);
				//MENSAJE
				//RM tiempo(2bytes) profundidad en mm (2 bytes) checksum
				checksum = COM_START_BYTE+COM_HALL_BYTE+tiempo+profundidad;
				itoh(tiempo, hex_tiempo, 2);
				itoh(profundidad, hex_profundidad, 2);
				itoh(checksum, hex_checksum, 2);
				serialPutchar(fd,COM_START_BYTE);
				serialPutchar(fd,COM_HALL_BYTE);
				serialPutchar(fd,hex_tiempo[0]);
				serialPutchar(fd,hex_tiempo[1]);
				serialPutchar(fd,hex_profundidad[0]);
				serialPutchar(fd,hex_profundidad[1]);
				serialPutchar(fd,hex_checksum[0]);
				serialPutchar(fd,hex_checksum[1]);
				printf("Pérdida motor\n");
				
				log_errores = fopen(name_log,"a");
				fprintf(log_errores, "%i %c : No hay lectura del encoder del motor.\n",tiempo,COM_HALL_BYTE);
				fclose(log_errores);
				
				//Vamos a intentar subir la sonda por si solo está fallando el encoder
				contador = 0;
				while(pulsador==0){ //mientras no se pulse, sube
					digitalWrite(in1,0);
					digitalWrite(in2,1);
					contador++;
					delay(100);
					//printf("%i\n",contador);
					if(contador>300){
						digitalWrite(in1,0);
						digitalWrite(in2,0); //se para el motor
						printf("Sonda perdida\n");
						//MENSAJE PERDIDA SONDA
						checksum = COM_START_BYTE+COM_LOSS_BYTE+tiempo;
						itoh(tiempo, hex_tiempo, 2);
						itoh(checksum, hex_checksum, 2);
						serialPutchar(fd,COM_START_BYTE);
						serialPutchar(fd,COM_LOSS_BYTE);
						serialPutchar(fd,hex_tiempo[0]);
						serialPutchar(fd,hex_tiempo[1]);
						serialPutchar(fd,hex_checksum[0]);
						serialPutchar(fd,hex_checksum[1]);
						
						log_errores = fopen(name_log,"a");
						fprintf(log_errores, "%i %c : La sonda no sube.\n",tiempo,COM_LOSS_BYTE);
						fclose(log_errores);
						
						en = 0;
						return;
					}
				}
				//Si ha conseguido subir tras la pérdida de vueltas
				
				digitalWrite(in1,1); // se baja un poco pero sin usar encoder porque no funciona
				digitalWrite(in2,0);
				delay(1000);
				digitalWrite(in1,0);
				digitalWrite(in2,0);
				
				//Mensaje de vuelta a casa segura
				checksum = COM_START_BYTE+COM_VUELTA_BYTE+tiempo;
				itoh(tiempo, hex_tiempo, 2);
				itoh(checksum, hex_checksum, 2);
				serialPutchar(fd,COM_START_BYTE);
				serialPutchar(fd,COM_VUELTA_BYTE);
				serialPutchar(fd,hex_tiempo[0]);
				serialPutchar(fd,hex_tiempo[1]);
				serialPutchar(fd,hex_checksum[0]);
				serialPutchar(fd,hex_checksum[1]);
				
				log_errores = fopen(name_log,"a");
				fprintf(log_errores, "%i %c : Vuelta segura a casa tras pérdida encoder motor.\n",tiempo,COM_VUELTA_BYTE);
				fclose(log_errores);
				
				en = 0;
				return;
			}
		}
		digitalWrite(in1,0); //para
		digitalWrite(in2,0);
		profundidad = (i+1)*p_d*1000;
		printf("Ha llegado a %i mm.\n",profundidad); //pone en pantalla a qué profundidad llegó
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
		
		printf("Mensaje: %X %X %X %X %X %X %X %X\n",COM_START_BYTE,PPZ_MEASURE_BYTE,hex_tiempo[0],hex_tiempo[1],
			hex_profundidad[0],hex_profundidad[1], hex_checksum[0], hex_checksum[1]);
		
		delay(t_estabilizacion); //espera tiempo necesario para estabilizar
		 
		i--;
	 }
	 //vuelta a cero
	 printf("Vuelta a cero.\n");
	 contador=0;
	 while(pulsador==0){ //mientras no se pulse, sube
		 digitalWrite(in1,0);
		 digitalWrite(in2,1);
		 contador++;
		 delay(100);
		 if(contador>=300){ //300*100=30 000 ms=30 s
			digitalWrite(in1,0);
			digitalWrite(in2,0);
			printf("Pérdida sonda.\n");
			//MENSAJE PERDIDA SONDA
			checksum = COM_START_BYTE+COM_LOSS_BYTE+tiempo;
			itoh(tiempo, hex_tiempo, 2);
			itoh(checksum, hex_checksum, 2);
			serialPutchar(fd,COM_START_BYTE);
			serialPutchar(fd,COM_LOSS_BYTE);
			serialPutchar(fd,hex_tiempo[0]);
			serialPutchar(fd,hex_tiempo[1]);
			serialPutchar(fd,hex_checksum[0]);
			serialPutchar(fd,hex_checksum[1]);
			
			log_errores = fopen(name_log,"a");
			fprintf(log_errores, "%i %c : La sonda no sube.\n",tiempo,COM_LOSS_BYTE);
			fclose(log_errores);
			
			en = 0;
			return;
		 }
	 }
	 
	 delay(1000); //espera 1s
	 
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
		
		 
	 //para no gastar se baja el sensor una vuelta de cuerda
	 while(vueltas<1){
		 digitalWrite(in1,1);
		 digitalWrite(in2,0);
	 }
	 digitalWrite(in1,0); //para
	 digitalWrite(in2,0);
		 
	 //Ping con la sonda
	 int a;
	 int g=0;
	 for(;;){
		 a = system("ping -c 1 8.8.8.8");
		 printf("PING %i %i\n",i,a);
		 g++;
		 delay(1000);
		 if(a==0){
			 break;
		 }
		 if(g>30){ //si en más de 30 intentos (30 s) no hemos conectado con el sensor
			printf("No se encuentra la sonda.\n");
			//MENSAJE PERDIDA SONDA
			checksum = COM_START_BYTE+COM_LOSS_BYTE+tiempo;
			itoh(tiempo, hex_tiempo, 2);
			itoh(checksum, hex_checksum, 2);
			serialPutchar(fd,COM_START_BYTE);
			serialPutchar(fd,COM_LOSS_BYTE);
			serialPutchar(fd,hex_tiempo[0]);
			serialPutchar(fd,hex_tiempo[1]);
			serialPutchar(fd,hex_checksum[0]);
			serialPutchar(fd,hex_checksum[1]);
			en = 0;
			return;
		 }
	 }

		 
	 
	 //Recogida de datos
	system("./prueba");
	 
	char data[200];
	int en_data = 0;
	 
	//Cambio de nombre al archivo de datos de la sonda, nombre archivo de datos de bajada
	char sensor_arch_name[100];
	char arch_name_datos_bajada[100];
	 
	t=time(NULL);
	tm=localtime(&t);
	strftime(sensor_arch_name,100,"Sensor-%X-%d-%m-%y.txt",tm); //se crea el nombre del archivo según fecha y hora
	strftime(arch_name_datos_bajada,100,"datos-bajada-%X-%d-%m-%y.txt",tm); //se crea el nombre del archivo según fecha y hora
	 
	//Leo los datos de la sonda y me guardo los datos que me interesan
	FILE *archivo_sonda = fopen("sensordata.txt","r"); //se abre el archivo para leer los datos
	FILE *archivo_datos_bajada = fopen(arch_name_datos_bajada,"w+"); //se crea el archivo para guardar los datos de la bajada
	
	fprintf(archivo_datos_bajada,"Date (ISO8601),System_Depth (m),Blue (µg/L),DO (mg/L),DO_SAT (%),pH_Value (pH),Temp (Cº)\n");
	
	while(!feof(archivo_sonda)){
		
		int year,month,day,hora,min,seg,data1,data2,data3,data4,data5,data6;
		
		fgets(data,200, archivo_sonda);
		
		sscanf(data,"%d-%d-%dT%d:%d:%d+0100,%d,%d,%d,%d,%d,%d\n",&year,&month,&day,&hora,&min,&seg,&data1,&data2,&data3,&data4,&data5,&data6);
		if(hora>=hora_bajada && min>=min_bajada||en_data==1){
			en_data=1;
			fprintf(archivo_datos_bajada,"%s\n",data);
		}
			
	}
	
	fprintf(archivo_datos_bajada, "\n \n Coord. GPS:\n Latitud %i\n Longitud %i\n Altitud %i\n",longitud,latitud,altitud); //añadir GPS
	
	fclose(archivo_sonda);
	fclose(archivo_datos_bajada);
	 
	 
	//se cambia al nombre creado para saber cuando se guardó por última vez los datos de la sonda
	rename("sensordata.txt", sensor_arch_name); 
	 
	 
	 printf("Fin de rutina.\n");
	 //RM tiempo(2bytes) profundidad en mm (2 bytes) checksum
	 checksum = COM_START_BYTE+PPZ_MEASURE_BYTE+tiempo+profundidad;
	 itoh(tiempo, hex_tiempo, 2);
	 itoh(profundidad, hex_profundidad, 2);
	 itoh(checksum, hex_checksum, 2);
	 serialPutchar(fd,COM_START_BYTE);
	 serialPutchar(fd,COM_FINAL_BYTE);
	 serialPutchar(fd,hex_tiempo[0]);
	 serialPutchar(fd,hex_tiempo[1]);
	 serialPutchar(fd,hex_profundidad[0]);
	 serialPutchar(fd,hex_profundidad[1]);
	 serialPutchar(fd,hex_checksum[0]);
	 serialPutchar(fd,hex_checksum[1]);
	 
	 
	 en=0; //Se desactiva el enable
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
	 
	 //CREAR ARCHIVOS DE DATOS
	 
	 //TELEMETRIA
	 //se crea el nombre del archivo según fecha y hora
	 strftime(name_telemetria,100,"Telemetria-%X-%d-%m-%y.txt",tm); 
	 
	 telemetria = fopen(name_telemetria,"w+"); //se crea el archivo de telemetria
	 while(telemetria==NULL){ //Por si hubiera ocurrido un error creando el archivo
		 telemetria = fopen(name_telemetria,"w+"); //se crea el archivo de telemetria
	 }
	 fprintf(telemetria, "TIME MENSAJE LATITUD LONGITUD ALTITUD PROFUNDIDAD CONFIANZA\n");
	 fclose(telemetria);
	 
	 //LOG ERRORES
	 strftime(name_telemetria,100,"log-errores-%X-%d-%m-%y.txt",tm); 
	 
	 log_errores = fopen(name_log,"w+"); //se crea el archivo de telemetria
	 while(log_errores==NULL){ //Por si hubiera ocurrido un error creando el archivo
		 log_errores = fopen(name_log,"w+"); //se crea el archivo de telemetria
	 }
	 fprintf(log_errores, "TIME MENSAJE\n");
	 fclose(log_errores);
	 
	 //PUNTOS EN LOS QUE SE HA MEDIDO
	 strftime(name_puntos_medida,100,"lorg-errores-%X-%d-%m-%y.txt",tm); 
	 
	 puntos_medida = fopen(name_puntos_medida,"w+"); //se crea el archivo de telemetria
	 while(puntos_medida==NULL){ //Por si hubiera ocurrido un error creando el archivo
		 puntos_medida = fopen(name_puntos_medida,"w+"); //se crea el archivo de telemetria
	 }
	 fprintf(puntos_medida, "TIME MENSAJE LATITUD LONGITUD ALTITUD PROFUNDIDAD CONFIANZA\n");
	 fclose(puntos_medida);
	 
	 //ARRANQUE HILOS
	 piThreadCreate(contador_vueltas);
	 piThreadCreate(contador_t);
	 piThreadCreate(comunicacion_ppzz);
	 
	 profundidad=0;
	 
	 delay(500);
	 
	 for(;;){
		 //printf("%i\n", en);
		 if(en==1){
			 rutina(d_sonar,intervalo);
		 }
		 if(en==0){
			 //printf("Esperando disparo\n");
		 }
		 delay(1000);
	 }
	 
	 return 0;
	 
 }
	 
