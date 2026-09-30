## TALLER CORTE 2 MICROS SAMUEL CASTRO
En este escrito se encontrará la explicación y evidencias de las tres actividades propuestas en el taller del segunto corte de la clase de micros.

# Control de desplazamiento de los drones por medio de la esp32 y consola de control:
Para esta actividad se hizo uso de 4 switches que permitían controlar el desplazamiento de los drones simulados en pybullet, donde cada boton indicaba una zona de ubicacion, al presionar un bot´rn la esp daba la orden a los drones de desplazarse hasta la zona asignada a este boton.
Cuando presionas A, B o C, envía ese comando por USB/Serial al computador. Python recibe el comando, cambia la posición objetivo de la formación y el controlador PID de gym-pybullet-drones mueve los drones de forma progresiva hasta esa zona.
A continuacion se adjunta el video de evidencia del funcionamiento: https://drive.google.com/file/d/1asNUj4RkRGPy7nwx1rVaF0Houf3x7uc3/view?usp=sharing
adicionalmente se adjunta los archivos de la programacion de la esp(esp_control_drones) y la generacion de los drones en pybullet(actividad_drones).

# Control de baxter
en la segunda activida del taller se pedia controlar el brazo de un baxter y ser capaz de agarrar y soltar un objeto con la garra del brazo, esto se hizo por medio de un joystick y una esp, a continuacion se muestra la evidencia del funcionamiento en un video y se encontarran los archivos de programacion del pybullet y la esp. Donde el moviemiento en x del joystick controlaba el movimiento lateral del brazo y el movimiento en Y controlaba la altura del brazo de baxter.
La ESP32 y el joystick permiten controlar el movimiento del brazo del Baxter. Los valores VRx y VRy se convierten en posiciones deseadas de la mano y, mediante cinemática inversa (IK), Python calcula los movimientos de las articulaciones. El botón del joystick controla el gripper, permitiendo agarrar, mover y soltar un objeto.
https://drive.google.com/file/d/1XGjR18sMDk7Hdhy75ELCBDNswPdM49t9/view?usp=sharing
# Control de Atlas:
En este numeral se pedía algo similar al punto anterior, pero ahora con el robot Atlas, un robot humanoide, se nos pedía controlar el desplazamiento fluido de este robot por medio de un joystick y la esp 32, mediante el joystick: VRy controla el avance y retroceso, mientras VRx controla el giro. Python recibe estas señales y aplica una rampa de aceleración, haciendo que el robot aumente y disminuya su velocidad progresivamente para obtener un movimiento más fluido. El botón permite detener y reanudar el movimiento.
A continuacion se adjunta el video de evidencia del funcionamiento del montaje fisico y los archivos de la programacion de la esp y el pybullet.
https://drive.google.com/file/d/1Wlnjw97K5tfjYqdFUcg1_WIObxpXTHo_/view?usp=sharing

