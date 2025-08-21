import RPi.GPIO as GPIO
import time

# Pines de los sensores de distancia
TRIGDER = 24
ECHODER = 23

TRIGIZQ = 17
ECHOIZQ = 27

# Pines del motor (puente H)
IN1 = 20
IN2 = 21

# Pin del servomotor
servo_pin = 12

# Variables
vueltas = 0
giros = 0

GPIO.setwarnings(False)
# Limpìeza de pines
GPIO.cleanup()
# Configuración digital
GPIO.setmode(GPIO.BCM)

# Configuración de los pines
# Sensores de dsitancia
GPIO.setup(TRIGDER, GPIO.OUT)
GPIO.setup(ECHODER, GPIO.IN)

GPIO.setup(TRIGIZQ, GPIO.OUT)
GPIO.setup(ECHOIZQ, GPIO.IN)
# Servomotores
GPIO.setup(servo_pin, GPIO.OUT)
# Motor de trtacción
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Inicializamos PWM a 50Hz (20ms) para servomotores
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0) # Empezamos sin mover el servo

# Función para colocar el grado del servomotor
def set_angle(angle):
    # Limita el ángulo entre 30° y 150°
    angle = max(30, min(150, angle))
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    print(f"Ángulo ajustado a {angle}° (Duty: {round(duty,2)}%)")
    time.sleep(0.05)  # Dar tiempo al servo a moverse
    
# Función para poder leer el sensor
def medir_distancia(TRIG, ECHO):
    GPIO.output(TRIG, False)
    time.sleep(0.5)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    timeout = time.time() + 1
    while GPIO.input(ECHO) == 0:
        if time.time() > timeout:
            print("Timeout esperando inicio de pulso")
            return None
        inicio = time.time()

    timeout = time.time() + 1
    while GPIO.input(ECHO) == 1:
        if time.time() > timeout:
            print("Timeout esperando fin de pulso")
            return None
        fin = time.time()

    duracion = fin - inicio
    distancia = duracion * 17150
    return round(distancia, 2)
# Función para activar movimiento del motor principal dependiendo la necesidad
def avanzar():
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)

def retro():
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
def stop():
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)

# EMPIEZA EL "VOID LOOP" O EL CÓDIGO PRINCIPAL
try:
    while True:
        ###############################################################################
        
        # Lectura derecha
        DER = medir_distancia(TRIGDER, ECHODER) 
        if DER:
            print(f"Distancia Derecha: {DER} cm")
        else:
            print("Error midiendo distancia derecha.")
        # Lectura izquierda
        IZQ = medir_distancia(TRIGIZQ, ECHOIZQ) 
        if IZQ:
            print(f"Distancia Izquierda: {IZQ} cm")
        else:
            print("Error midiendo distancia Izquierda.")
        # Tiempo de espera entre lecturas
        time.sleep(0.06)
        
        ###############################################################################
        
        if vueltas < 3:
        # Movimiento de servomotor en relación a los sensores
            if DER > 110:# Espacio encontrado para girar a la derecha
                set_angle(30)
                avanzar()
                time.sleep(6)
                set_angle(110)
                time.sleep(1)
                giros += 1
            elif IZQ > 110: # Espacio encontrado para girar a la izquierda
                set_angle(150)
                avanzar()
                time.sleep(6)
                set_angle(110)
                time.sleep(1)
                giros += 1
            else: # Seguir avanzando debido a paredes a los costados
                set_angle(110)
            
            # Control de motor de tracción trasera
            avanzar()
            vueltas = giros / 4
            print("Giros: {}, Vueltas: {}".format(giros, vueltas))
            
        ###############################################################################
            
        elif vueltas == 3:
            set_angle(110)
            stop()
            
except KeyboardInterrupt:
    GPIO.cleanup()
    print("Programa terminado.")
