# MicroRobotica

## Descripción General

**MicroRobotica** es el repositorio de trabajo de **Alejandro Aguirre Diaz** (Autor) de la clase con el mismo nombre. En este repositorio se documenta todo el proceso de aprendizaje en **ROS 2** (Robot Operating System 2), desde la configuración inicial en un codespace de GitHub hasta la implementación de un sistema distribuido de comunicación entre nodos.

Este trabajo sirve como referencia completa para aprender los principios de desarrollo de robótica moderna con ROS 2.

## 📝 Actividades de Clase

### Actividad I. Creación del entorno de trabajo
Configuración del Workspace de ROS 2 Jazzy en un entorno de desarrollo basado en contenedores (Codespaces) y creación de los primeros paquetes de comunicación:

* **[Paquete `sensor_pkg`](src/sensor_pkg):** Implementación básica de un nodo publicador de temperatura.
* **[Paquete `conv_pkg`](src/conv_pkg):** Nodo suscriptor que realiza la conversión de datos a grados Celsius y los vuelve a publicar.

---

### Actividad II. Tópicos - Robot Planar RR
Modelado de la cinemática de un brazo robótico de dos grados de libertad (RR) basándose en parámetros Denavit-Hartenberg (DH). Todo el desarrollo se encuentra en el siguiente paquete:

* **[Paquete `robot_kinematics`](src/robot_kinematics):** Contiene la lógica de movimiento y cálculo espacial.
    * **[Nodo Publicador de Articulaciones (`joint_publisher.py`)](src/robot_kinematics/robot_kinematics/joint_publisher.py):** Genera una trayectoria dinámica para los ángulos $q_1$ y $q_2$, oscilando entre 0 y 180 grados ($0$ a $\pi$ rad) de forma continua con logs en grados sexagesimales.
    * **[Nodo de Cinemática Directa (`forward_kinematics.py`)](src/robot_kinematics/robot_kinematics/forward_kinematics.py):** Suscriptor que recibe los ángulos, aplica las ecuaciones de transformación basadas en la longitud de los eslabones ($l_1$ y $l_2$) y calcula la posición cartesiana $(x, y)$ del extremo final.

**Descripción del conjunto:** El sistema simula el movimiento de un robot físico en tiempo real. Un nodo actúa como el "driver" de los motores (publicando estados de juntas), mientras que el otro actúa como el "procesador geométrico" que traduce esos movimientos angulares en una posición espacial útil, permitiendo monitorear el espacio de trabajo del robot de manera dinámica.
### 🚀 Guía de Ejecución y Resultados

Para poner en marcha el sistema de cinemática, abre tres terminales independientes y ejecuta los siguientes comandos:

#### **Terminal 1: Generador de Movimiento (`joint_pub`)**
Este nodo inicia la simulación enviando los ángulos de las articulaciones.
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_kinematics joint_pub
```
Aqui se mostrara una lista continua de los ángulos $q_1$ y $q_2$. Observarás cómo los valores aumentan hasta llegar a 180° y luego decrementan hasta 0°, mostrando la conversión de radianes a grados en tiempo real.

#### **Terminal 2: Solucionador Cinemático (`fk_solver`)**
Este nodo realiza los cálculos matemáticos basados en la tabla DH.
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_kinematics fk_solver
```
Se vera el log del nodo procesando cada mensaje recibido de las juntas y transformándolo en coordenadas cartesianas. Verás mensajes tipo: Extremo en -> X: 0.750, Y: 0.230.

#### **Terminal 3: Monitor de Datos (`echo`)**
Herramienta de introspección para verificar el tópico de salida.
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /robot_position
```
Se mostrara la estructura pura del mensaje geometry_msgs/Point. Es la mejor forma de validar que los datos se están transmitiendo correctamente a través del middleware de ROS 2.

### Actividad III: Servicio (Cinemática Inversa)

#### Descripción
En esta actividad se implementó una arquitectura Cliente-Servidor en ROS 2 para calcular la **Cinemática Inversa** de un brazo robótico planar de 2 grados de libertad (2R). El robot asume longitudes de eslabón de $l_1 = 1.0$ y $l_2 = 1.0$.

Para la comunicación, se definió un servicio personalizado llamado `MoveArm.srv` dentro del paquete `interface_robotics`:

* **[Definición del servicio (`MoveArm.srv`)](src/interface_robotics/srv/MoveArm.srv)**

```text
# Petición (Request) - Coordenadas objetivo
float64 x
float64 y
---
# Respuesta (Response) - Ángulos articulares calculados
float64 q1
float64 q2
```

#### Nodos del Sistema (`robot_kinematics`)

* **[Servidor de Cinemática Inversa (`ik_server.py`)](src/robot_kinematics/robot_kinematics/ik_server.py):** actúa como el cerebro matemático. Recibe las coordenadas espaciales $(X, Y)$ y calcula los ángulos articulares $(q_1, q_2)$ con trigonometría. Además, valida límites del espacio de trabajo; si el punto es inalcanzable, reporta error y responde de forma segura sin colapsar el nodo.
* **[Cliente de Cinemática Inversa (`ik_client.py`)](src/robot_kinematics/robot_kinematics/ik_client.py):** nodo asíncrono de consola. Toma los argumentos de posición, envía la petición al servidor, espera la respuesta y muestra los resultados en radianes.

#### Instrucciones de Ejecución
Para probar el sistema, usa dos terminales distintas y ejecuta en ambas el entorno de ROS 2 y del workspace.

1. Iniciar el servidor (Terminal 1)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_kinematics ik_server
```

El servidor quedará a la espera con el mensaje: `Servidor de Cinematica Inversa listo para recibir coordenadas (X, Y)...`.

2. Enviar una petición desde el cliente (Terminal 2)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_kinematics ik_client 1.0 1.0
```

#### Prueba de Estrés: Mapeo Automático del Espacio de Trabajo
Para comprobar los límites físicos del robot y la robustez del servidor, se puede usar un script de Bash que genera una matriz de coordenadas (de -2.0 a 2.0). Ejecuta este bloque con el servidor activo en otra terminal:

```bash
for x in $(seq -2.0 0.5 2.0); do
    for y in $(seq -2.0 0.5 2.0); do
        echo "======================================="
        echo "Solicitando coordenadas X=$x, Y=$y"
        ros2 run robot_kinematics ik_client "$x" "$y"
        sleep 0.2
    done
done
```

Este barrido permite observar cómo el servidor resuelve los puntos válidos y rechaza de forma controlada los puntos matemáticamente inalcanzables.

### Actividad VII: Seguimiento de Trayectoria Circular con Control PD + Compensación de Gravedad

#### Descripción
En esta actividad se modeló en MATLAB el comportamiento dinámico de un robot planar de 2 GDL sometido a una ley de control PD con compensación de gravedad. El script [ActividadVII_23110162_7E.m](MATLAB/Scripts/ActividadVII_23110162_7E.m) integra la dinámica con `ode45`, reconstruye la posición cartesiana del extremo del manipulador por cinemática directa y genera una referencia circular de radio fijo para evaluar el seguimiento.

La salida se resume en la imagen [Actividad VII_23110162_7E.png](MATLAB/Imagenes/Actividad%20VII_23110162_7E.png), donde se comparan la trayectoria deseada y la trayectoria real, además de las curvas de posición y velocidad articular.

#### Análisis de resultados
El controlador logra llevar al manipulador hacia la órbita de referencia y mantener un seguimiento estable durante la mayor parte de la simulación. En la gráfica cartesiana se observa un transitorio inicial pronunciado, especialmente en el arranque del movimiento, pero después la trayectoria real se acopla de forma razonable al círculo deseado. Las señales articulares muestran un movimiento suave y físicamente consistente, con un máximo esfuerzo al inicio por la corrección del error inicial y luego una evolución periódica acorde con la referencia.

El resultado confirma que la compensación de gravedad ayuda a reducir la carga de la ley PD, aunque todavía existe desviación en la fase transitoria y pequeñas diferencias geométricas entre la referencia y la trayectoria obtenida.

### Actividad VIII: Comparación de Leyes de Control y Cálculo del Índice L2

#### Descripción
En esta actividad se compararon tres estrategias de control para seguir una trayectoria circular: PD, TANH y ATAN. El script [ActividadVIII_23110162_7E.m](MATLAB/Scripts/ActividadVIII_23110162_7E.m) resuelve la dinámica para cada caso, reconstruye la trayectoria cartesiana real, calcula el error articular instantáneo y evalúa un índice de desempeño tipo RMS/L2 para cuantificar qué controlador sigue mejor la referencia.

La figura correspondiente, [Actividad VIII_23110162_7E.png](MATLAB/Imagenes/Actividad%20VIII_23110162_7E.png), muestra la comparación de trayectorias, posiciones y velocidades articulares, la evolución del error y las barras del índice L2.

#### Análisis de resultados
De acuerdo con la gráfica de barras, el mejor desempeño lo obtiene el controlador PD con un índice L2 de aproximadamente 0.2273 rad, mientras que TANH y ATAN presentan errores mayores, cercanos a 0.3092 rad y 0.3014 rad, respectivamente. Esto indica que, para este caso, la ley PD ofrece la mejor precisión global de seguimiento.

En las trayectorias cartesianas, los tres controladores siguen el círculo de forma aceptable, pero TANH y ATAN muestran una desviación ligeramente mayor en algunos tramos. La ventaja de los controladores no lineales aparece más en la suavidad de las señales y en la limitación de esfuerzos, aunque no superan al PD en exactitud para esta prueba concreta. La evolución del error confirma esta tendencia: el PD reduce antes la discrepancia angular y converge con menor error acumulado.

### Actividad IX: Control Cartesiano en Espacio Operacional

#### Descripción
En esta actividad se implementó un control en espacio operacional para un brazo planar de 2 GDL. El script [ActividadIX_23110162_7E.m](MATLAB/Scripts/ActividadIX_23110162_7E.m) construye una trayectoria circular en coordenadas cartesianas, interpola la referencia temporal y aplica dinámica cartesiana basada en el Jacobiano para gobernar el movimiento articular. La salida visualiza la trayectoria deseada vs. la trayectoria real, así como las posiciones y velocidades angulares.

La evidencia gráfica se encuentra en [Actividad IX_23110162_7E.png](MATLAB/Imagenes/Actividad%20IX_23110162_7E.png).

#### Análisis de resultados
El seguimiento cartesiano reproduce la forma general de la trayectoria circular, pero se nota una aproximación más discreta y con segmentos rectos cuando el número de puntos de referencia es reducido. Esto hace visible la dependencia entre la calidad de la interpolación y la suavidad del movimiento real. Aun así, el brazo converge a la zona de interés y mantiene coherencia geométrica con la referencia.

Las gráficas articulares muestran cambios grandes al inicio, lo que refleja la necesidad de reconfigurar el manipulador desde una postura cercana a la singularidad inicial. En términos generales, el control cartesiano demuestra ser efectivo para imponer la trayectoria en el espacio operacional, aunque su desempeño final depende fuertemente de la discretización de la referencia, de las ganancias usadas y de la condición inicial del robot.

## Requisitos del Sistema

### Versión del Sistema Operativo
- **Ubuntu 24.04.3 LTS** (Servidor de larga duración)
  - **Justificación técnica**: Ubuntu 24.04 es la versión LTS más reciente y proporciona soporte a largo plazo (5 años) hasta 2029. Esta versión es compatible con ROS 2 Jazzy, ofreciendo compatibilidad de bibliotecas modernas, seguridad actualizada y kernels optimizados para sistemas embebidos y robóticos.

### Versión de ROS 2
- **ROS 2 Jazzy Jalisco** (o compatible con formato de paquete ament_python)
  - **Justificación técnica**: ROS 2 utiliza una arquitectura basada en DDS (Data Distribution Service), proporcionando comunicación más eficiente y determinista. El formato de paquete ament_python permite crear nodos Python modernos con gestión automática de dependencias mediante setuptools.


### Requisitos de Desarrollo
- Python 3.12+ (incluido en Ubuntu 24.04)
- colcon-core (herramienta de construcción multipaquete)
- rclpy (cliente de ROS 2 para Python)
- ament-python (sistema de construcción para paquetes Python)

## Arquitectura y Componentes

### 1. Paquete `sensor_pkg` - Sistema de Sensores

#### Nodo: `pub_temp` (Publicador de Sensor)
**Archivo**: [src/sensor_pkg/sensor_pkg/pub_temp.py](src/sensor_pkg/sensor_pkg/pub_temp.py)

- **Funcionalidad**: Simula lectura de sensor publicando datos concatenados
- **Tópico**: `sensor` (tipo: `std_msgs/String`)
- **Formato de mensaje**: Cadena "temperatura" + índice incremental
- **Frecuencia**: 2 Hz (período de 0.5 segundos)

**Justificación técnica**:
- Uso de `std_msgs/String` para datos de configuración o diagnóstico
- Mayor frecuencia (2 Hz) respecto a otros publicadores para simular sensor de mayor tasa de muestreo
- Implementa el patrón de productor de datos

#### Nodo: `subs_temp` (Subscriptor de Sensor)
**Archivo**: [src/sensor_pkg/sensor_pkg/subs_temp.py](src/sensor_pkg/sensor_pkg/subs_temp.py)

- **Funcionalidad**: Recibe y procesa datos del sensor
- **Tópico**: `sensor` (tipo: `std_msgs/String`)
- **Procesamiento**: Imprime datos en consola para diagnóstico y validación


---

### 2. Paquete `conv_pkg` - Conversión de Temperaturas

#### Nodo: `pub_fahrenheit` (Publicador)
**Archivo**: [src/conv_pkg/conv_pkg/pub_fahrenheit.py](src/conv_pkg/conv_pkg/pub_fahrenheit.py)

- **Funcionalidad**: Publica valores de temperatura en escala Fahrenheit cada segundo
- **Tópico**: `temp_f` (tipo: `std_msgs/Float32`)
- **Rango inicial**: 70°F incrementándose 1°F por cada ciclo
- **Frecuencia**: 1 Hz (1 mensaje por segundo)

#### Nodo: `conv_celsius` (Subscriptor y Publicador)
**Archivo**: [src/conv_pkg/conv_pkg/conv_celsius.py](src/conv_pkg/conv_pkg/conv_celsius.py)

- **Funcionalidad**: Recibe temperaturas en Fahrenheit, convierte a Celsius y publica el resultado
- **Tópico entrada**: `temp_f` (tipo: `std_msgs/Float32`)
- **Tópico salida**: `temp_c` (tipo: `std_msgs/Float32`)
- **Algoritmo de conversión**: $C = (F - 32) \times \frac{5}{9}$

**Justificación técnica**:
- Implementa el patrón de procesamiento de datos (nodo intermediario)
- Usa callbacks para procesamiento orientado a eventos
- Desacoplamiento de productores y consumidores mediante tópicos
- Demuestra pipeline de transformación de datos

### Patrones de Arquitectura Implementados

1. **Patrón Publish-Subscribe (Pub-Sub)**
   - Desacoplamiento espacio-temporal entre productores y consumidores
   - Escalabilidad horizontal (múltiples suscriptores para un publicador)

2. **Patrón de Procesamiento de Flujo de Datos**
   - `pub_fahrenheit` → `conv_celsius` → (consumidores finales)
   - Pipeline de transformación de datos

3. **Patrón de Nodo Único (Single Responsibility)**
   - Cada nodo tiene una responsabilidad clara
   - Facilita pruebas y reutilización

---

## Creación del Workspace desde Cero

Esta sección es una **guía didáctica paso a paso** para crear un workspace de ROS 2 desde cero.

### Paso 1: Preparar el Codespace de GitHub

```bash
# 1. Crear un nuevo codespace en GitHub (o usar uno existente)
# 2. Abrir una terminal en el codespace
# 3. Actualizar paquetes del sistema
sudo apt-get update && sudo apt-get upgrade -y

# 4. Instalar dependencias de ROS 2 (si no están preinstaladas)
sudo apt-get install -y python3-colcon-common-extensions
```

### Paso 2: Crear la estructura del Workspace

```bash
# 1. Navegar al directorio home
cd ~

# 2. Crear el workspace
mkdir -p MicroRobotica_ws/src
cd MicroRobotica_ws

# 3. Verificar la estructura creada
ls -la
# Deberías ver: src/  (y posteriormente build/, install/, log/)
```

### Paso 3: Crear el Paquete `sensor_pkg`

```bash
# 1. Navegar a la carpeta src
cd ~/MicroRobotica_ws/src

# 2. Crear el paquete Python
ros2 pkg create --build-type ament_python sensor_pkg

# 3. Navegar al paquete creado
cd sensor_pkg/sensor_pkg

# 4. Crear los nodos
# a) Crear pub_temp.py (publicador de sensor)
cat > pub_temp.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PubSensor(Node):
    def __init__(self):
        super().__init__('pub_tem')
        self.publisher=self.create_publisher(String, 'sensor',10)
        period=0.5
        self.timer=self.create_timer(period,self.callback)
        self.i=0
    def callback(self):
        msg=String()
        msg.data="temperatura"+str(self.i)
        self.publisher.publish(msg)
        self.i+=1
        print(msg)

def main(arg=None):
    rclpy.init(args=arg)
    pub_sensor=PubSensor()
    rclpy.spin(pub_sensor)
    pub_sensor.destroy_node()
    rclpy.shutdown()
    print('Hi from sensor_pkg.')

if __name__ == '__main__':
    main()
EOF

# b) Crear subs_temp.py (subscriptor de sensor)
cat > subs_temp.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubsSensor (Node):
    def __init__(self):
        super().__init__('subs_sensor')
        self.subscriber = self.create_subscription(String, 'sensor', self.callback, 10)
    
    def callback(self,msg):
        print(msg.data)

def main(args=None):
    rclpy.init(args=args)
    subs_sensor=SubsSensor()
    rclpy.spin(subs_sensor)
    subs_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### Paso 4: Configurar el Paquete `sensor_pkg`

```bash
# 1. Editar setup.py para agregar los entry points
# Navega a: ~/MicroRobotica_ws/src/sensor_pkg/
# Abre setup.py y dentro de entry_points, agrega:
#    'pub_temp = sensor_pkg.pub_temp:main',
#    'subs_temp = sensor_pkg.subs_temp:main'

# 2. Tu setup.py debería verse así (sección entry_points):
cat >> setup.py << 'EOF'
    entry_points={
        'console_scripts': [
            'pub_temp = sensor_pkg.pub_temp:main',
            'subs_temp = sensor_pkg.subs_temp:main'
        ],
    },
EOF
```

### Paso 5: Crear el Paquete `conv_pkg`

```bash
# 1. Navegar de nuevo a src
cd ~/MicroRobotica_ws/src

# 2. Crear el paquete
ros2 pkg create --build-type ament_python conv_pkg

# 3. Navegar al directorio de nodos
cd conv_pkg/conv_pkg

# 4. Crear pub_fahrenheit.py (publicador de temperaturas)
cat > pub_fahrenheit.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PubFahrenheit(Node):
    def __init__(self):
        super().__init__('pub_fahrenheit')
        self.publisher_ = self.create_publisher(Float32, 'temp_f', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.f_val = 70.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.f_val
        self.publisher_.publish(msg)
        self.get_logger().info(f'Enviando: {msg.data} °F')
        self.f_val += 1.0

def main(args=None):
    rclpy.init(args=args)
    node = PubFahrenheit()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# 5. Crear conv_celsius.py (convertidor Fahrenheit a Celsius)
cat > conv_celsius.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ConvCelsius(Node):
    def __init__(self):
        super().__init__('conv_celsius')
        self.subscription = self.create_subscription(Float32, 'temp_f', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32, 'temp_c', 10)

    def listener_callback(self, msg):
        f = msg.data
        c = (f - 32) * 5 / 9
        msg_c = Float32()
        msg_c.data = float(c)
        self.publisher_.publish(msg_c)
        self.get_logger().info(f'Recibido: {f}°F -> Convertido: {c:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = ConvCelsius()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### Paso 6: Configurar el Paquete `conv_pkg`

```bash
# 1. Navega a ~/MicroRobotica_ws/src/conv_pkg/
# 2. Edita setup.py y en entry_points agrega:
#    'talker_f = conv_pkg.pub_fahrenheit:main',
#    'converter_c = conv_pkg.conv_celsius:main'

cat >> setup.py << 'EOF'
    entry_points={
        'console_scripts': [
            'talker_f = conv_pkg.pub_fahrenheit:main',
            'converter_c = conv_pkg.conv_celsius:main',
        ],
    },
EOF
```

---

## Compilación y Construcción

### Paso 1: Compilar el Workspace Completo

```bash
# 1. Navegar al workspace
cd ~/MicroRobotica_ws

# 2. Compilar con colcon
colcon build

# Salida esperada:
# Starting >>> sensor_pkg
# Finished <<< sensor_pkg [0.XXs]
# Starting >>> conv_pkg
# Finished <<< conv_pkg [0.XXs]
# Summary: 2 packages built
```

**¿Qué hace colcon build?**
- Detecta automáticamente los paquetes en `src/`
- Resuelve dependencias entre paquetes
- Compila en paralelo para mayor velocidad
- Genera los scripts de configuración en `install/`

### Paso 2: Verificar la Compilación

```bash
# 1. Lista los paquetes instalados
ls -la install/

# Deberías ver: conv_pkg/  sensor_pkg/  (además de otros archivos)

# 2. Verifica la estructura de instalación
ls install/sensor_pkg/
ls install/conv_pkg/
```

### Paso 3: Cargar el Entorno

**IMPORTANTE**: Debes hacer esto en **cada terminal nueva** que abras.

```bash
# Opción 1: Usar bash
source ~/MicroRobotica_ws/install/setup.bash

# Opción 2: Usar zsh (si lo prefieres)
source ~/MicroRobotica_ws/install/setup.zsh

# Verificar que funciona
echo $ROS_PACKAGE_PATH
# Debería incluir: /home/codespace/MicroRobotica_ws/install
```

**Justificación técnica**:
- `colcon build` es la herramienta estándar de ROS 2 para construir múltiples paquetes
- Soporta compilación paralela y gestión de dependencias entre paquetes
- Genera artefactos en `build/`, `install/` y `log/`
- El script setup.bash configura las variables de entorno necesarias para que ROS 2 encuentre los paquetes

## Ejecución

Las siguientes instrucciones muestran de forma didáctica cómo compilar (si hace falta) y ejecutar los paquetes en varias terminales. Se asume que el workspace está en `~/MicroRobotica_ws`.

Nota: antes de ejecutar los nodos, en cada terminal debes cargar el entorno de ROS 2 Jazzy y el `setup` del workspace.


### sensor_pkg (primero)

Compilación (si es necesario):
```bash
# Terminal: compilar (si hace falta)
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Terminal 1
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run sensor_pkg pub_temp
```

Terminal 2
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run sensor_pkg subs_temp
```

Terminal 3
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /sensor
```


### conv_pkg (después)

Terminal 1
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run conv_pkg talker_f
```

Terminal 2
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run conv_pkg converter_c
```

Terminal 3
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /temp_c
```

## Herramientas de Diagnóstico

### Ver nodos activos
```bash
ros2 node list
```

### Ver tópicos disponibles
```bash
ros2 topic list
ros2 topic info /nombre_topico
```

### Inspeccionar datos en tiempo real
```bash
ros2 topic echo /temp_f
ros2 topic echo /temp_c
```

### Información de nodos
```bash
ros2 node info /nombre_nodo
```

### Grafo de comunicación (requiere graphviz)
```bash
ros2 run rqt_graph rqt_graph
```
