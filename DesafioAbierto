//inclusión de las librerias a utilizar
#include <Servo.h>

//configuracion de pines de los sensores de distancia
#define trigPin2 7
#define trigPin3 6
#define trigPin4 10
#define echoPin2 9
#define echoPin3 8
#define echoPin4 11

int pulsadorPin = 5; // Variable que guarda el número del pin al que conectamos el pulsador.
int valorPulsador = 0; // Variable donde almacenaremos la lectura del estado del pulsador.
int k; //valor que almacenara si se pulso el botón o no

//Configuracion de los pines de los motores (cc y servo)
#define motorPin1 2
#define motorPin2 4
#define servoPin 3

//nombre del serovmotor
Servo direccion;

//todas las variables para los sensores de dirrecion
long duration, distance;

long durationAde = 0;
long durationAtras = 0;
long durationIzq = 0;
long durationDerch = 0;

long distanceAde= 0;
long distanceAtras = 0;
long distanceIzq = 0;
long distanceDerch = 0;

//variables para los if
int a = 0;
int i = 0;
int z= 0;
int d;
int motor;



//variables para el conteo de vueltas
int giro;
int vuelta;

void setup() {
  //inicialización de todos los pines
  Serial.begin(9600);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  direccion.attach(servoPin);

  pinMode(pulsadorPin, INPUT_PULLUP); // Declaramos la variable inputPin como entrada y activamos su resistencia interna Pullup.
}

void loop() {
  valorPulsador = digitalRead(pulsadorPin);  // Lectura digital de pulsadorPin
  if (valorPulsador == HIGH) {
    k = k;
  }
  else {
    k = 1;
  }
  //si el pulsador fue activado entonces ejecutar el código
  if(k == 1){
  //leer las distancia de los sensores
  leerdistancia();
  //contar cuantas vueltas he dado en base a los giros realizando una división
  vuelta = giro/4;
  Serial.print("giros: ");
  Serial.print(giro);
  Serial.print("  vueltas:");
  Serial.println(vuelta);
    //si no  ha completado las 3 vueltas entonces...
   if (vuelta < 3){
  //si no hay pared del lado derecho entonces girar hacia la derecha
    if(distanceDerch>120){
    direccion.write(0);
    avanzar();
    delay(4000);
    adelante();
    delay(3000);
    giro = giro + 1;
    distanceDerch=0;
    distanceIzq= 0;
    }
  //si hay escpacio hacia la izquierda entonces girar hacia la izquierda
  if (distanceIzq>120) {
    if(vuelta == 0){
    direccion.write(180);
    avanzar();
    delay(3400);
    adelante();
    delay(3000);
    giro = giro + 1;
    distanceDerch=0;
    distanceIzq= 0;
    Serial.println("izquierda");
    }
    if(vuelta == 1){
      direccion.write(180);
    avanzar();
    delay(3500);
    adelante();
    delay(3000);
    giro = giro + 1;
    distanceDerch=0;
    distanceIzq= 0;
    Serial.println("izquierda");
    }
    if(vuelta == 2){
    direccion.write(180);
    avanzar();
    delay(3400);
    adelante();
    delay(3000);
    giro = giro + 1;
    distanceDerch=0;
    distanceIzq= 0;
    Serial.println("izquierda");
    }
  }
  /*
  //si esta muy pegado hacia la derecha entonces centrarse
  else if (distanceDerch < 20){
    direccion.write(110);
    avanzar();
    vueltaContada = false; // Resetea la variable cuando se cambia el estado
  }
  //si esta muy pegado hacia la izquierda entonces centrarse
  else if (distanceIzq < 20){
    direccion.write(80);
    avanzar();
    vueltaContada = false; // Resetea la variable cuando se cambia el estado
  }*/
  //si esta centrado que siga derecho
   else{
    adelante(); 
   }
  }
 //si se completaron las vueltas que se posicione al comienzo y que se finalize el progama al terminar
 if (vuelta == 3){
  stop();
  digitalWrite(motorPin1,LOW);
  digitalWrite(motorPin2,LOW);
 }
 }
}

//lectura de las 3 distancias e imprimirlas en el monitor serial
void leerdistancia(){
  // Sensor 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration = pulseIn(echoPin2, HIGH);
  distance = duration * 0.034 / 2;

  distanceDerch = distance;
  
  Serial.print("Sensor 2: ");
  Serial.print(distanceDerch);
  Serial.print(" cm");

  Serial.print("   ");
  
  duration = 0;
  distance = 0;

  // Sensor 3
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration = pulseIn(echoPin3, HIGH);
  distance = duration * 0.034 / 2;
  distanceIzq = distance;
  
  Serial.print("Sensor 3: ");
  Serial.print(distanceIzq);
  Serial.print(" cm"); 

  Serial.println("   ");
  
  duration = 0;
  distance = 0;
}

//rectificar las llantas y avanzar
void adelante(){
  direccion.write(95); // Posición del servo para avanzar

  digitalWrite(motorPin1,LOW);
  digitalWrite(motorPin2,HIGH);
}

//solamente avanzar con la posicion de llantas anteriormente establecida en el código
void avanzar(){
  digitalWrite(motorPin1,LOW);
  digitalWrite(motorPin2,HIGH);
}

//detenerse totalmente
void stop(){
  digitalWrite(motorPin1,LOW);
  digitalWrite(motorPin2,LOW);
}
