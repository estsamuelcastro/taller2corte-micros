## TALLER CORTE 2 MICROS SAMUEL CASTRO
En este escrito se encontrará la explicación y evidencias de las tres actividades propuestas en el taller del segunto corte de la clase de micros.

# Control de desplazamiento de los drones por medio de la esp32 y consola de control:
Para esta actividad se hizo uso de 4 switches que permitían controlar el desplazamiento de los drones simulados en pybullet, donde cada boton indicaba una zona de ubicacion, al presionar un bot´rn la esp daba la orden a los drones de desplazarse hasta la zona asignada a este boton.
Cuando presionas A, B o C, envía ese comando por USB/Serial al computador. Python recibe el comando, cambia la posición objetivo de la formación y el controlador PID de gym-pybullet-drones mueve los drones de forma progresiva hasta esa zona.
A continuacion se adjunta el video de evidencia del funcionamiento: https://drive.google.com/file/d/1asNUj4RkRGPy7nwx1rVaF0Houf3x7uc3/view?usp=sharing
adicionalmente se adjunta los archivos de la programacion de la esp(esp_control_drones) y la generacion de los drones en pybullet(actividad_drones).
