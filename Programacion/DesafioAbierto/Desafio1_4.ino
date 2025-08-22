#include <Servo.h>
#include <Wire.h>
#include <math.h> // Necesario para funciones matematicas como atan2 y sqrt

const int MPU_ADDR = 0x68; // Direccion I2C del MPU-9250

// Constantes de escala para conversion de datos brutos a unidades fisicas
// Sensibilidad del acelerometro para un rango de ±2g (por defecto)
const float ACCEL_SCALE = 16384.0;
// Sensibilidad del giroscopio para un rango de ±250°/s (por defecto)
const float GYRO_SCALE = 131.0;
// Factor de peso para el filtro complementario.
// Un valor mas alto (cercano a 1) da mas peso al giroscopio.
// Un valor mas bajo (cercano a 0) da mas peso al acelerometro.
// 0.98 es un buen equilibrio para la mayoria de aplicaciones de vehiculos.
const float alpha = 0.98;

// Variables para la calibracion del giroscopio
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Variables para el angulo fusionado por el filtro complementario
float angleX = 0, angleY = 0, angleZ = 0; // angleZ sigue siendo solo por integracion de giroscopio
unsigned long previousTime; // Para calcular el tiempo transcurrido (deltaTime)

// Pines de sensores de distancia (ultrasonicos)
#define trigPin2 7  // Sensor derecho
#define trigPin3 6  // Sensor izquierdo
#define trigPin4 22 // Sensor frontal
#define echoPin2 9
#define echoPin3 8
#define echoPin4 24

// Pines del motor y servo
#define motorPin1 2
#define motorPin2 4
#define servoPin 3 

// Pin del boton
int pulsadorPin = 5; // Pin del pulsador
int valorPulsador = 0; // Almacena la lectura del estado del pulsador
int k = 0; // Valor que indicara si se pulso el boton o no (0 = no pulsado, 1 = pulsado)

// Variables para el conteo de giros y vueltas (logica del carro)
int giro = 0;
int vuelta = 0;

int giroZInicial = 0;
const int ANCHO_CARRO = 16;  // cm

long duration2, duration3, duration4; // Duracion de pulso para sensores ultrasonicos
long distance2, distance3, distance4; // Distancia calculada por sensores ultrasonicos

// Para definir la direccion de los giros fijos del carro
// 0: Sin direccion definida (inicio)
// 1: Girar siempre a la izquierda
// 2: Girar siempre a la derecha
int direccionGiros = 0; 

Servo direccion; // Objeto del servomotor

// Funcion para medir distancia con filtro de validez
long medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distancia = duration * 0.034 / 2; // Conversion a cm

  // Filtro simple para asegurar que la distancia es razonable
  if (distancia >= 0 && distancia <= 400) {
    return distancia;
  } else {      return -1; // Valor invalido o fuera de rango
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Inicializa la comunicacion I2C

  // Despertar el MPU-9250
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registro PWR_MGMT_1 (Power Management 1)
  Wire.write(0x00); // Escribir 0x00 para sacar el sensor del modo de reposo
  Wire.endTransmission(true);

  // Calibrar el giroscopio para establecer el punto cero
  Serial.println("Calibrando MPU-9250. No mover el sensor...");
  calibrateGyro(); // Llama a la funcion de calibracion
  Serial.println("Calibracion completa.");
  
  // Establecer los angulos iniciales de Pitch (X) y Roll (Y) usando el acelerometro
  // Esto es crucial para que el filtro complementario empiece con una buena estimacion
  readAccelAndSetInitialAngle();

  previousTime = micros(); // Guarda el tiempo actual para calcular deltaTime en el loop
  
  // Configuracion de pines de los sensores ultrasonicos
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(trigPin4, OUTPUT);

  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT);

  // Configuracion de pines de los motores
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // Inicializar el objeto servomotor
  direccion.attach(servoPin);

  // Configuracion del pin del pulsador
  // INPUT_PULLUP activa la resistencia interna, el pin estara HIGH por defecto.
  // Se detecta pulsacion cuando el pin va a LOW.
  pinMode(pulsadorPin, INPUT_PULLUP); 
}

void mantenerCentro(int distIzq, int distDer) {
  // Calcular ancho de la pista (pared a pared)
  int anchoPista = distIzq + distDer + ANCHO_CARRO;

  // Margen ideal desde cada pared
  int margenIdeal = (anchoPista - ANCHO_CARRO) / 2;

  // Desviación actual respecto al centro
  int error = distIzq - distDer;

  // Si el error es pequeño -> recto
  if (abs(error) < 5) {
    adelante();
  }
  // Pegado a la derecha
  else if (distIzq < margenIdeal) {
    girarSuaveDerecha();
  }
  // Pegado a la izquierda
  else if (distDer < margenIdeal) {
    girarSuaveIzquierda();
  }
}

void girarSuaveDerecha() {
  direccion.write(70);
  avanzar();
}

void girarSuaveIzquierda() {
  direccion.write(100);
  avanzar();
}

// Funcion para mover el carro hacia adelante
void adelante(){
  direccion.write(88); // Posicion del servo para avanzar (ajustar segun tu servo)
  digitalWrite(motorPin1,LOW); // Control del motor (ejemplo: LOW para un pin, HIGH para otro)
  digitalWrite(motorPin2,HIGH);
}

// Funcion para avanzar con la posicion de llantas ya establecida
void avanzar(){
  digitalWrite(motorPin1,LOW);
  digitalWrite(motorPin2,HIGH);
}

// Funcion para detener el carro completamente
void stop(){
  digitalWrite(motorPin1,LOW);
  digitalWrite(motorPin2,LOW);
}

// Funcion para girar a la derecha
void girarDerecha() {
  while (angleZ>giroZInicial-85){
    direccion.write(40);
    avanzar();
    lecturaMPU();
  }
  adelante(); // Regresar el servo al centro (ajustar)
  delay(1000); // Tiempo para estabilizar despues del giro
}

// Funcion para girar a la izquierda
void girarIzquierda() {
  while (angleZ<giroZInicial+85){
    direccion.write(140);
    avanzar();
    lecturaMPU();
  }
  adelante(); // Regresar el servo al centro (ajustar)
  delay(1500); // Tiempo para estabilizar despues del giro
}

// Funcion para calibrar el giroscopio al inicio
void calibrateGyro() {
  const int numSamples = 1000; // Numero de muestras para promediar
  long sumX = 0, sumY = 0, sumZ = 0; // Sumas para calcular el promedio de bias
  
  for (int i = 0; i < numSamples; i++) {
    // Iniciar la transmision I2C al MPU
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Registro GYRO_XOUT_H (primer registro del giroscopio X)
    Wire.endTransmission(false); // No liberar el bus despues de escribir
    Wire.requestFrom(MPU_ADDR, 6, true); // Pedir 6 bytes (X, Y, Z para el giroscopio)
    
    // Leer los bytes y combinarlos en valores de 16 bits
    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

    sumX += rawGyroX;
    sumY += rawGyroY;
    sumZ += rawGyroZ;
    delay(2); // Pequeño retraso entre lecturas
  }
  
  // Calcular el promedio y convertirlo a grados/segundo
  // Este promedio es la compensacion (bias) que se restara de futuras lecturas
  gyroBiasX = ((float)sumX / numSamples) / GYRO_SCALE;
  gyroBiasY = ((float)sumY / numSamples) / GYRO_SCALE;
  gyroBiasZ = ((float)sumZ / numSamples) / GYRO_SCALE;
}

// Funcion para leer el acelerometro y establecer los angulos iniciales de Pitch y Roll
void readAccelAndSetInitialAngle() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Registro ACCEL_XOUT_H (primer registro del acelerometro X)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Pedir 6 bytes (X, Y, Z para el acelerometro)
  
  int16_t accelX_raw = Wire.read() << 8 | Wire.read();
  int16_t accelY_raw = Wire.read() << 8 | Wire.read();
  int16_t accelZ_raw = Wire.read() << 8 | Wire.read();
  
  // Convertir valores brutos a 'g' (unidades de gravedad)
  float accelX = (float)accelX_raw / ACCEL_SCALE;
  float accelY = (float)accelY_raw / ACCEL_SCALE;
  float accelZ = (float)accelZ_raw / ACCEL_SCALE;
  
  // Calcular los angulos iniciales de Pitch (X) y Roll (Y) basados en el acelerometro
  // atan2 es mas robusto que atan y maneja todos los cuadrantes
  angleX = atan2(accelY, accelZ) * 180 / M_PI; // Pitch (rotacion alrededor del eje X)
  angleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI; // Roll (rotacion alrededor del eje Y)
  // El eje Z (Yaw) no puede ser determinado con el acelerometro, se deja en 0 inicialmente
  angleZ = 0; 
}

void lecturaMPU(){
    // Obtener el tiempo transcurrido desde la ultima lectura para la integracion del giroscopio
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - previousTime) / 1000000.0; // En segundos
  previousTime = currentTime;

  // ==== LECTURA DEL MPU-9250 (ACELEROMETRO Y GIROSCOPIO) ====
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Iniciar la lectura desde ACCEL_XOUT_H (0x3B)
  Wire.endTransmission(false);
  // Pedir 14 bytes: AccelX (2), AccelY (2), AccelZ (2), Temp (2), GyroX (2), GyroY (2), GyroZ (2)
  Wire.requestFrom(MPU_ADDR, 14, true); 

  // Leer y combinar los bytes del acelerometro
  int16_t accelX_raw = Wire.read() << 8 | Wire.read();
  int16_t accelY_raw = Wire.read() << 8 | Wire.read();
  int16_t accelZ_raw = Wire.read() << 8 | Wire.read();

  // Saltarse los 2 bytes de temperatura (0x41 y 0x42)
  Wire.read(); Wire.read();

  // Leer y combinar los bytes del giroscopio
  int16_t gyroX_raw = Wire.read() << 8 | Wire.read();
  int16_t gyroY_raw = Wire.read() << 8 | Wire.read();
  int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();
  
  // Convertir valores brutos del acelerometro a 'g'
  float accelX = (float)accelX_raw / ACCEL_SCALE;
  float accelY = (float)accelY_raw / ACCEL_SCALE;
  float accelZ = (float)accelZ_raw / ACCEL_SCALE;

  // Convertir valores brutos del giroscopio a grados/segundo y restar el bias
  float gyroX = (float)gyroX_raw / GYRO_SCALE - gyroBiasX;
  float gyroY = (float)gyroY_raw / GYRO_SCALE - gyroBiasY;
  float gyroZ = (float)gyroZ_raw / GYRO_SCALE - gyroBiasZ;

  // Calcular el angulo a partir del acelerometro (Pitch y Roll)
  // Estos angulos son mas estables a largo plazo pero sensibles a la aceleracion lineal
  float accelAngleX = atan2(accelY, accelZ) * 180 / M_PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;
  
  // ==== APLICAR FILTRO COMPLEMENTARIO ====
  // Para los ejes X (Pitch) y Y (Roll), el filtro combina giroscopio y acelerometro
  angleX = alpha * (angleX + gyroX * deltaTime) + (1.0 - alpha) * accelAngleX;
  angleY = alpha * (angleY + gyroY * deltaTime) + (1.0 - alpha) * accelAngleY;

  // Para el eje Z (Yaw), el acelerometro no puede proporcionar una referencia de gravedad.
  // El angulo Z se sigue calculando solo por integracion del giroscopio y, por lo tanto, seguira derivando.
  // Para corregir la deriva del eje Z, se necesitaria un magnetometro.
  angleZ += gyroZ * deltaTime;

  /*
  // Imprimir los angulos para los tres ejes
  Serial.print("Angulo X: "); Serial.print(angleX);
  Serial.print("  |  Angulo Y: "); Serial.print(angleY);
  Serial.print("  |  Angulo Z: "); Serial.print(angleZ);
  Serial.print(" | Delta Time: "); Serial.println(deltaTime * 1000.0); // Mostrar en ms
  */
  // Pequeño retraso para la lectura de los sensores de distancia
  delay(10); 
}

void loop() {
  // ==== LECTURA DE SENSORES DE DISTANCIA ====
  distance2 = medirDistancia(trigPin2, echoPin2); // Derecha
  distance3 = medirDistancia(trigPin3, echoPin3); // Izquierda
  distance4 = medirDistancia(trigPin4, echoPin4); // Frente

  //Lectura del sensor giroscopio, eje Z a tomar en cuenta
  lecturaMPU();

  Serial.print("  Derecha: ");
  if (distance2 != -1)
    Serial.print(distance2);
  else
    Serial.print("Invalido");
  Serial.print(" cm\t");

  Serial.print("Izquierda: ");
  if (distance3 != -1)
    Serial.print(distance3);
  else
    Serial.print("Invalido");
  Serial.print(" cm\t");

  Serial.print("Frente: ");
  if (distance4 != -1)
    Serial.print(distance4);
  else
    Serial.print("Invalido");
  Serial.println(" cm");
  

  // ==== LOGICA DE CONTROL DEL CARRO ====
  valorPulsador = digitalRead(pulsadorPin); // Lectura del estado del pulsador
  if (valorPulsador == LOW) { // Si el pulsador esta presionado (LOW por INPUT_PULLUP)
    k = 1; // Activar la logica de movimiento del carro
  }
  
  if(k == 1){ // Si el boton fue activado
    // Calcular vueltas (asumiendo 4 giros completos para una vuelta)
    vuelta = giro / 4;
    Serial.print("Giros: ");
    Serial.print(giro);
    Serial.print("  Vueltas: ");
    Serial.println(vuelta);

    if (vuelta < 3){ // Mover el carro por 3 vueltas
      // Lógica para definir la dirección de giros al inicio
      if (direccionGiros == 0) {
        if (distance3 != -1 && distance3 > 120) { // Si hay espacio a la izquierda
          direccionGiros = 1; // Elegir girar siempre a la izquierda
          Serial.println(">> Direccion de giro elegida: IZQUIERDA");
        } else if (distance2 != -1 && distance2 > 120) { // Si hay espacio a la derecha
          direccionGiros = 2; // Elegir girar siempre a la derecha
          Serial.println(">> Direccion de giro elegida: DERECHA");
        }
      }

      // Ejecutar el giro segun la direccion definida y si hay espacio
      if (direccionGiros == 1 && distance3 != -1 && distance3 > 120) {
        Serial.println("Giro a la izquierda ejecutado");
        giroZInicial = angleZ;
        girarIzquierda();
        giro++; // Contar el giro
        // Resetear distancias para evitar giros multiples por la misma deteccion
        distance2 = 0; 
        distance3 = 0;
      } else if (direccionGiros == 2 && distance2 != -1 && distance2 > 120) {
        Serial.println("Giro a la derecha ejecutado");
        giroZInicial = angleZ;
        girarDerecha();
        giro++; // Contar el giro
        // Resetear distancias
        distance2 = 0;
        distance3 = 0;
      } 
      mantenerCentro(distance3, distance2);
      delay(15); // Pequeño retraso para controlar la velocidad
    } else { // Si se completaron 3 vueltas
      stop(); // Detener el carro
      digitalWrite(motorPin1,LOW); // Asegurar que los motores estan apagados
      digitalWrite(motorPin2,LOW);
    }
  }
}