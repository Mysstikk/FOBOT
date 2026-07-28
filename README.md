# FOBOT — Feeding Collaborative Robot

FOBOT es un prototipo de robot colaborativo de seis grados de libertad diseñado para asistir durante la alimentación de personas con movilidad reducida o diversidad funcional.

El sistema combina actuadores Dynamixel, ROS 2, MoveIt 2 y visión artificial para ejecutar una secuencia de recogida de comida y aproximar una cuchara a la boca del usuario mediante control visual.

Este repositorio contiene el código fuente desarrollado para el Trabajo de Fin de Grado:

> *Design and implementation of a collaborative robot for feeding with control through artificial vision*

Autor: Unai Mollá Caballero  
Tutor: Jaime Masiá Vañó  
Universitat Politècnica de València — Campus d'Alcoi  
Curso: 2025–2026

## Características principales

- Control de seis actuadores Dynamixel.
- Integración del hardware mediante `ros2_control`.
- Ejecución de trayectorias articulares previamente configuradas.
- Grabación de posiciones mediante guiado manual.
- Coordinación del comportamiento mediante una máquina de estados finitos.
- Detección facial mediante MediaPipe.
- Reconocimiento de la apertura de la boca y el cierre de los ojos.
- Estimación tridimensional de la posición de la boca.
- Aproximación mediante MoveIt Servo.
- Supervisión de singularidades y colisiones.
- Detección de posibles contactos mediante el esfuerzo de los motores.
- Interfaz física mediante pulsadores e indicadores LED.

## Arquitectura del repositorio

```text
FOBOT/
├── robot_pkg/                 # Control principal, FSM y movimientos
├── fobot_moveit_config/       # Configuración de MoveIt 2
├── fobot_vision/              # Detección facial y control visual
├── singularity_probe/         # Análisis del jacobiano
├── DynamixelSDK               # SDK de los motores Dynamixel
├── dynamixel_interfaces/      # Mensajes y servicios de Dynamixel
├── dynamixel_hardware_interface/
│                              # Integración con ros2_control
└── README.md
```

La estructura puede variar dependiendo de la versión publicada.

## Requisitos

El sistema se ha desarrollado utilizando:

- Ubuntu 22.04.
- ROS 2 Humble.
- Python 3.
- C++.
- MoveIt 2.
- `ros2_control`.
- Dynamixel SDK.
- OpenCV.
- MediaPipe.
- Cámara USB.
- Raspberry Pi 4.

También se necesitan los paquetes de ROS 2 correspondientes a MoveIt Servo, TF2, `usb_cam` y el controlador de trayectorias articulares.

## Instalación

Crear un espacio de trabajo de ROS 2:

```bash
mkdir -p ~/fobot_ws/src
cd ~/fobot_ws/src
```

Clonar el repositorio:

```bash
git clone <URL_DEL_REPOSITORIO>
```

Instalar las dependencias:

```bash
cd ~/fobot_ws
rosdep install --from-paths src --ignore-src -r -y
```

Compilar:

```bash
colcon build --symlink-install
source install/setup.bash
```

La instrucción `source install/setup.bash` debe ejecutarse en cada nueva terminal desde la que se utilice el robot.

## Configuración previa

Antes de iniciar el sistema se debe comprobar:

1. Que cada motor Dynamixel tenga un identificador diferente.
2. Que el puerto serie y la velocidad de comunicación sean correctos.
3. Que la cámara esté conectada y calibrada.
4. Que el plato y el robot estén situados en las posiciones utilizadas durante la configuración.
5. Que la cuchara esté limpia y correctamente fijada.
6. Que no haya obstáculos dentro del espacio de trabajo.

Los parámetros principales se encuentran en los archivos YAML de los paquetes correspondientes.

## Ejecución

Para iniciar el sistema completo:

```bash
ros2 launch robot_pkg FOBOT.launch.py
```

El fichero maestro inicia los controladores, la descripción del robot, la máquina de estados, el sistema de visión y MoveIt Servo.

Después del arranque se debe realizar el movimiento de *homing* antes de comenzar la secuencia de alimentación.

## Grabación de trayectorias

El movimiento automático utiliza una secuencia de posiciones articulares almacenada en un fichero CSV.

El programa de grabación permite:

1. Desactivar el par de los motores.
2. Mover manualmente el brazo.
3. Registrar las posiciones deseadas.
4. Volver a activar el par.
5. Probar la trayectoria antes de utilizarla con comida.

Las primeras pruebas deben realizarse sin situar al usuario dentro del alcance inmediato del robot.

## Funcionamiento general

La secuencia de alimentación es la siguiente:

1. El robot realiza el movimiento de *homing*.
2. El usuario abre la boca para iniciar el ciclo.
3. El robot ejecuta la trayectoria de recogida.
4. La cuchara se desplaza hasta una posición próxima al rostro.
5. El usuario vuelve a abrir la boca para confirmar la entrega.
6. El sistema de visión estima la posición de la boca.
7. MoveIt Servo realiza la aproximación final.
8. El robot regresa para recoger una nueva porción.
9. El usuario puede finalizar el proceso cerrando ambos ojos.

## Limitaciones

El prototipo presenta actualmente las siguientes limitaciones:

- No detecta automáticamente la comida presente en el plato.
- La recogida utiliza trayectorias previamente grabadas.
- El plato debe permanecer en una posición aproximadamente fija.
- La profundidad se estima mediante una cámara RGB monocular.
- El comportamiento puede verse afectado por la iluminación y las oclusiones.
- No se ha realizado una validación clínica ni una certificación de seguridad.
- El sistema debe utilizarse en un entorno controlado y con supervisión.

## Seguridad

Este software pertenece a un prototipo académico y no constituye un producto médico o asistencial certificado.

Antes de utilizarlo:

- Mantener despejado el espacio de trabajo.
- No introducir las manos mientras los motores estén activos.
- Comprobar la temperatura y consistencia de la comida.
- Verificar la fijación de la cuchara.
- No rearmar el sistema sin identificar la causa de la parada.
- Utilizar el interruptor de alimentación si el robot presenta un comportamiento inesperado.

El uso del código y del prototipo se realiza bajo la responsabilidad de la persona que lo instala y opera.

Unai Mollá Caballero  
