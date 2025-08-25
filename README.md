# Eje-404
Somos un equipo de WRO Future Engineers 2025 de El Salvador. En este proyecto empleamos mecánica, electrónica y robótica para poder completar todas las fases de esta categoría. En este proyecto se utilizó diseño 3D con impresión de filamento PLA, Arduino Mega 2560, placa de expansión para sensores, baterías 18650, reductores de voltaje, engranajes, etc.


Los integrantes de este grupo somos:

- María José Guirola         - Miembro del equipo
- Mauricio Andrés Valladares - Miembro del equipo
- Rocío Elizabeth Ortiz      - Coach del equipo


# Movilidad del carro 
Este carro contiene 2 motores. El primero motor es un motor DC con caja reductora que trabaja de 5 a 9 v y 1.5 amperios que maneja su avance y retroceso, con una tracción trasera. El motor tiene un engranaje en su eje que no es perfectamente circular, más conocido como eje tipo D o D shaft, para que pueda generar torque con el engranaje sin que se deslice o resbale. El engranaje se conecta con otro idéntico a este, que está unido a un eje largo estriado el cual contiene las llantas a sus extremos. Logrando así, un sistema de transmisión por engranaje. Al tener el mismo tamaño y cantidad de dientes se mantiene la misma velocidad y torque, sin embargo, la dirección de giro se invierte por el principio de inversión del sentido de giro en engranajes. El segundo motor es un servomotor mg90s, color negro y de engranajes metálicos. Este controla la dirección delantera del automóvil. Se adaptó en 2 partes cruciales para replicar la dirección Ackermann en función de nuestro espacio y posibilidades.

La primera pieza, que simula una barra de acoplamiento es un prisma rectangular horizontal con un hueco rectangular vertical al medio, para incrustar una pieza metálica que hace de votante con un pico que apunta hacia el suelo y hace el juego para el movimiento. El movimiento es transmitido por el servomotor que está atornillado a esta placa.
La segunda parte son las portamanguetas o knuckles, sujetan a las llantas y les transmiten la dirección. Estas piezas se asemejan a un L, ya que poseen 3 kingpins, 2 paralelos para unirse al chasis del carro y otro para unirse a la barra de acoplamiento y con un hueco cilíndrico para el eje independiente de cada rueda. Cuando la barra se desplaza hacia un costado, debido a los kingping y el límite que les coloca el chasis, ellas giran sobre su propio eje, replicando este giro a las llantas.


En conclusión, con esta simplificación de la dirección de Ackermann logramos una dirección bastante exacta en un espacio reducido y con un sistema de transmisión por engranaje conseguimos una tracción con un solo motor, y la fuerza de torque y velocidad igual que la de la salida del motor.


# Alimentación del circuito
El circuito se alimenta únicamente por 2 porta pilas. Ambos poseen dos pilas 18650 en serie, con 4.2 voltios y 2800 mAh cada una, dándonos 8.4v y 2800 mAh cada porta pilas aproximadamente. Debido a que todos los sensores, servomotor y Arduino soportan hasta 5v, se utilizaron 2 reguladores de voltaje XL4015 para tener diferentes voltajes (8.4v, 5v y 3.3v).


A continuación, mostraremos una tabla donde indica el amperaje y voltaje que cada componente necesita y de que fuente es alimentado. Cabe recalcar que se unieron todas las tierras para tener un GND común y estabilidad en el circuito.

| Nombre   | Cantidad | Amperaje  | Amperaje Total  | Voltaje  | Porta pilas |
|---|---|---|---|---|---|
|Motor DC  | 1  | 1.5A  | 1.5A  | 8.4v  | B  |
|Servomotor mg90s   | 1  | 2.7mA  | 2.7 mA  | 5v  | A  |
|SR-HC04  | 3  | 15mA  | 45mA  | 5v  | A  |
|Pixy 2.1  | 1  | 140mA  | 140mA  | 5v  | A  |
|L298n  | 1  | 250mA  | 250mA  | 8.4v  | B  |
|MPU9250  | 1  | 3.2mA  | 3.2mA  | 3.3v  | B  |
|Sensor Shield  | 1  | 40mA  | 40mA  | 5v  | A  |
|Arduino Mega 2560  | 1  | 40mA  | 40mA  | 5v  | A  |
|Switch A  | 1  | -  | -  | 8.4v  | A  |
|Switch B  | 1  | -  | -  | 8.4v  | B  |
|XL4015 (A)  | 1  | -  | -  | 8.4v  | A  |
|XL4015 (B)  | 1  | -  | -  | 8.4v  | B  |


# Uso de los sensores
