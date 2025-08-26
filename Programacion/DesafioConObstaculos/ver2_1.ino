#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo direccion;

// Definir los límites del servo
#define SERVO_MIN 40
#define SERVO_MAX 140
#define SERVO_CENTER 88 
#define SERVO_PIN 3

// Estados del sistema de evasión

//Los estados son una forma de organizar el comportamiento del código, se le llama una Máquina de Estados Finita (FSM - Finite State Machine).
//En lugar de escribir todo en loop() con if y else por todos lados, se divide el comportamiento en bloques bien definido, pueden definirse dentro del loop o como funciones y solo llamarlas en el loop.

enum EstadoEvasion {
  BUSCANDO_OBJETO,         // Busca si hay bloques bloque
  ESQUIVANDO_OBJETO,       // Mueve el servo para esquivar
  RECENTRAR_TRAYECTORIA,   // Vuelve al centro y esperar
  ESPERANDO_SIGUIENTE      // Busca el siguiente bloque
};

EstadoEvasion estadoActual = BUSCANDO_OBJETO; // Estado inicial del sistema
unsigned long tiempoInicioCentrado = 0;       // Tiempo para temporizar el centrado

// Calculos

// Función para calcular la distancia en cm con base en el ancho del bloque detectado
float calcularDistanciaCM(int ancho) {
  if (ancho <= 0) return 1000; // Si el ancho es inválido, retorna distancia muy grande
  float y = (float)ancho;
  float x = ((1.0 / y) + 0.003758) / 0.001099;
  return x;
}

// Función para calcular el ángulo del servo con el control proporcional
int calcularAnguloServo(int x_actual, int x_ideal, float proporcion) {
  int error = x_ideal - x_actual;
  float giro = error * proporcion;
  int servo = SERVO_CENTER + (int)giro;

  // Limitar ángulo para no dañar al carro
  if (servo < SERVO_MIN) servo = SERVO_MIN;
  if (servo > SERVO_MAX) servo = SERVO_MAX;

  return servo;
}

// === FUNCIÓN PRINCIPAL DE DETECCIÓN Y CONTROL ===

// Detecta el bloque más cercano (firma 1 o 2), calcula su posición ideal y mueve el servo
bool controlarPixyYServo() {
  pixy.ccc.getBlocks();

  // Si no hay bloques, no continua
  if (pixy.ccc.numBlocks == 0) {
    Serial.println("No se detectaron bloques.");
    return false;
  }

  // Variables para guardar el bloque más cercano válido
  int mejorIndex = -1;
  float mejorDistancia = 10000;
  int firmaElegida = 0;

  // Buscar cual es el bloque que esta mas cerca, teniendo el cuenta la distancia
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    int firma = pixy.ccc.blocks[i].m_signature;

    if (firma != 1 && firma != 2) continue; // Ignorar bloques que no sean rojo o verde

    int ancho = pixy.ccc.blocks[i].m_width;
    float distancia = calcularDistanciaCM(ancho);

    // Define cual de los bloques es al que se le dara prioridad
    if (distancia < mejorDistancia) {
      mejorDistancia = distancia;
      mejorIndex = i;
      firmaElegida = firma;
    }
  }

  // Si no hay bloques válidos, salir
  if (mejorIndex == -1) {
    Serial.println("No hay bloques válidos.");
    return false;
  }

  // Datos del bloque más cercano
  int x_actual = pixy.ccc.blocks[mejorIndex].m_x;
  int ancho = pixy.ccc.blocks[mejorIndex].m_width;
  float distancia = calcularDistanciaCM(ancho);

  float proporcion = 0.25; // Proporción por defecto
  int x_ideal = 160;       // Valor por defecto (si no hay firma válida)

  // === Lógica de evasión según distancia ===
  //si esta a menos de 20 cm que retroceda
  if (distancia < 20.0) {
    Serial.println("Retrocediendo...");
    return false;
  } 
  //Si esta en el rango de 21 a 45 cm, una proporcion mas brusca y una x ideal mas a los extremos
  else if (distancia >= 21.0 && distancia <= 45.0) {
    proporcion = 0.5;
    //el ? es un operador ternario, es una forma compacta de poner if-else, SOLO USAR PARA DECISIONES SENCILLAS, si son mas complicadas usar if-else, esta es su sintaxis ---- condición ? valor_si_verdadero : valor_si_falso;
    x_ideal = (firmaElegida == 1) ? 39 : 274; // si el bloque es rojo, el ideal es 39, si es verde es 274
  } 
  //Si esta a 46 o mas lejos una proporcion mas suave y una x ideal mas centrada
  else if (distancia > 46.0) {
    proporcion = 0.25;
    x_ideal = (firmaElegida == 1) ? 77 : 235; // si el bloque es rojo, el ideal es 77, si es verde es 235
  }

  // Calcular y aplicar el ángulo del servo
  int anguloServo = calcularAnguloServo(x_actual, x_ideal, proporcion);
  direccion.write(anguloServo);

  // Imprimir datos
  Serial.print("Firma: ");
  Serial.print(firmaElegida);
  Serial.print(" | x_actual: ");
  Serial.print(x_actual);
  Serial.print(" | x_ideal: ");
  Serial.print(x_ideal);
  Serial.print(" | Distancia: ");
  Serial.print(distancia);
  Serial.print(" cm | Proporción: ");
  Serial.print(proporcion);
  Serial.print(" | Servo: ");
  Serial.println(anguloServo);

  // Verificar si ya se alineó correctamente, si el actual es mayor menor que el idea para el rojo, o mayor que el ideal para el verde, lo toma como bien posicionado
  if ((firmaElegida == 1 && x_actual <= x_ideal) || (firmaElegida == 2 && x_actual >= x_ideal)) {
    return true;
  }

  return false;
}

// === FUNCIONES POR ESTADO ===

// Estado: Buscar un nuevo bloque
void estadoBuscandoObjeto() {
  static bool bloqueDetectado = false;
  bloqueDetectado = controlarPixyYServo();
  if (bloqueDetectado) {
    Serial.println(">> Objeto detectado. Iniciando evasión...");
    estadoActual = ESQUIVANDO_OBJETO;
  }
}

// Estado: Esquivar el bloque alineando el servo
void estadoEsquivandoObjeto() {
  static bool bloqueDetectado = false;
  bloqueDetectado = controlarPixyYServo();
  if (!bloqueDetectado) {
    Serial.println(">> Objeto esquivado. Recentrando...");
    direccion.write(SERVO_CENTER);         // Volver al centro
    tiempoInicioCentrado = millis();       // Guardar tiempo actual
    estadoActual = RECENTRAR_TRAYECTORIA;  // Pasar al siguiente estado
  }
}

// Estado: Mantener servo al centro por 1 segundo
void estadoRecentrarTrayectoria() {
  if (millis() - tiempoInicioCentrado >= 1000) { // Sigue recto durante 1 segundo
    Serial.println(">> Reanudando búsqueda de siguiente bloque...");
    estadoActual = ESPERANDO_SIGUIENTE;
  }
}

// Estado: Esperar a que aparezca el siguiente bloque
void estadoEsperandoSiguiente() {
  static bool bloqueDetectado = false;
  bloqueDetectado = controlarPixyYServo();
  if (bloqueDetectado) {
    Serial.println(">> Nuevo objeto detectado. Iniciando nueva evasión...");
    estadoActual = ESQUIVANDO_OBJETO;
  }
}

// === SETUP Y LOOP PRINCIPAL ===

void setup() {
  Serial.begin(115200); 
  Serial.println("Iniciando Pixy2 + Servo con lógica secuencial...");

  direccion.attach(SERVO_PIN);
  direccion.write(SERVO_CENTER);

  pixy.init();
}

void loop() {
  // En cada ciclo, se ejecutan las funciones, si se quiere modificar que hace cada uno, se modifica las funciones, y solo se referencian aqui
  switch (estadoActual) {
    case BUSCANDO_OBJETO:
      estadoBuscandoObjeto();
      break;

    case ESQUIVANDO_OBJETO:
      estadoEsquivandoObjeto();
      break;

    case RECENTRAR_TRAYECTORIA:
      estadoRecentrarTrayectoria();
      break;

    case ESPERANDO_SIGUIENTE:
      estadoEsperandoSiguiente();
      break;
  }

  delay(100);
}
