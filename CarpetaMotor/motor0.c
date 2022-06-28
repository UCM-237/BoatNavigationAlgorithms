#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include <stdio.h>

#define ENC_ERROR 0 //valores de error como 0

int hall1_pin = 0; //pin lectura hall 1, cable azul
int hall2_pin = 1; //pin lectura hall 2, cable amarillo

int pulsador_pin = 2; //pin lectura pulsador de posición cero

int in1 = 3; //pines control giro
int in2 = 4;

int PWM_pin = 12 ; //pin control velocidad

int inc, previoA, previoB, actualA,actualB, ind,pulsos, vueltas,giro, pulsador,d;

int M_inc[16]={0,-1,1,0,
	1,ENC_ERROR,ENC_ERROR,-1,
	-1,ENC_ERROR,ENC_ERROR,1,
	0,1,-1,0}; //matriz de pulsos de encoder según salidas hall
 //+1 pulso sentido horario
 //-1 pulso sentido antihorario
 //0 no hay movimiento
 
void fgiro(int giro){
	 if(giro==1){
		 digitalWrite(in1,0);
		 digitalWrite(in2,0);
		 delay(100);
		 digitalWrite(in1,0);
		 digitalWrite(in2,1);
	 }
	 if(giro==-1){
		 digitalWrite(in1,0);
		 digitalWrite(in2,0);
		 delay(100);
		 digitalWrite(in1,1);
		 digitalWrite(in2,0);
	 }
	 if(giro==0){
		 digitalWrite(in1,0);
		 digitalWrite(in2,0);
	 }
 }
 
 PI_THREAD(contador_vueltas){
	 pulsador = 0;
	 pulsos = 0; //inicializar a cero
	 vueltas = 0;
	 actualA = digitalRead(hall1_pin); //primera lectura
	 actualB = digitalRead(hall2_pin);
	 
	 for(;;){
		 previoA = actualA; //actualización
		 previoB = actualB;
		 
		 //pulsador = digitalRead(pulsador_pin);
		 while(pulsador==1){ //sonda en posición cero
			 fgiro(0);
			 pulsos = 0; //inicializa a cero el encoder
			 vueltas = 0;
			 pulsador = digitalRead(pulsador_pin);
		 }
		 
		 actualA = digitalRead(hall1_pin); //lectura actual
		 actualB = digitalRead(hall2_pin);
		 ind = 8*previoA+4*actualA+2*previoB+actualB; //cálculo índice matriz
		 inc = M_inc[ind]; //incremento 
		 pulsos +=inc; //pulsos totales 
		 vueltas = pulsos/98; //cálculo vueltas
	 }
 }
 
 
 
 void rutina_bajada(int d){ //bajar hasta d (m) parando en intervalos de 1m
	 while(pulsador==0){ //comprobación de que la sonda está en posición cero
		 fgiro(-1); //subida hasta que se encuentre en cero
	 }
	 fgiro(0); //parada
	 delay(2000); //espera 2s
	 printf("Sensor en posición cero.\n");
	 
	 int vec_vueltas[10] = {5,10,15,20,25,30,35,40,45,50}; //vueltas para llegar a cada posición
	 int vec_dist[10] = {1,2,3,4,5,6,7,8,9,10}; // diferentes distancias a elegir

	 int i;
	 for(i=0;i<d;i++){
		 
		 while(vueltas<vec_vueltas[i]){
			 fgiro(1);
		 }
		 fgiro(0);
		 delay(10000); //espera 10s
		 printf("Ha llegado a %i m.\n",vec_dist[i]);
	 
	 }
	 
 }
 
  void rutina_subida(){
	 while(pulsador==0){ //mientras no se pulse, sube
		 fgiro(-1);
	 }
	 delay(2000); //espera 2s
	 printf("Sensor en posición cero.\n");
 }
 
 void main(){
	 wiringPiSetup();
	 pinMode(hall1_pin,INPUT);
	 pinMode(hall2_pin,INPUT);
	 pinMode(pulsador_pin,INPUT);
	 pinMode(in1, OUTPUT);
	 pinMode(in2,OUTPUT);
	 pinMode(PWM_pin,PWM_OUTPUT);
	 
	 digitalWrite(in1,0);
	 digitalWrite(in2,0);

	 delay(6000);

	 
 }
	 
