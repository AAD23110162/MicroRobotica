# MicroRobotica

## Descripción General

**MicroRobotica** es el repositorio de trabajo de **Alejandro Aguirre Diaz** (Autor) de la clase con el mismo nombre. En este repositorio se documenta todo el proceso de aprendizaje en **ROS 2** (Robot Operating System 2), desde la configuración inicial en un codespace de GitHub hasta la implementación de un sistema distribuido de comunicación entre nodos.

El proyecto implementa:
- **Subsistema de sensores**: Publicación y recepción de datos de sensores
- **Subsistema de conversión de temperaturas**: Procesamiento en pipeline de transformación de unidades
- **Arquitectura publish-subscribe**: Desacoplamiento entre productores y consumidores de datos
- **Conceptos fundamentales de ROS 2**: Nodos, tópicos, mensajes y comunicación distribuida en sistemas de tiempo real

Este trabajo sirve como referencia completa para aprender los principios de desarrollo de robótica moderna con ROS 2.

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

## Estructura del Proyecto

```
MicroRobotica_ws/
├── src/
│   ├── sensor_pkg/                  # Paquete de sensores
│   │   ├── sensor_pkg/
│   │   │   ├── __init__.py
│   │   │   ├── pub_temp.py          # Nodo publicador de sensor
│   │   │   └── subs_temp.py         # Nodo subscriptor de sensor
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── conv_pkg/                    # Paquete de conversión de temperaturas
│       ├── conv_pkg/
│       │   ├── __init__.py
│       │   ├── conv_celsius.py      # Nodo convertidor Fahrenheit → Celsius
│       │   └── pub_fahrenheit.py    # Nodo publicador de temperaturas
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
├── build/                           # Artefactos de compilación
├── install/                         # Paquetes instalados
├── log/                             # Registros de compilación
└── README.md
```

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

**Justificación técnica**:
- Implementa el patrón de consumidor de datos
- Callbacks síncronos garantizan procesamiento ordenado
- Ideal para depuración y monitoreo en tiempo real

---

### 2. Paquete `conv_pkg` - Conversión de Temperaturas

#### Nodo: `pub_fahrenheit` (Publicador)
**Archivo**: [src/conv_pkg/conv_pkg/pub_fahrenheit.py](src/conv_pkg/conv_pkg/pub_fahrenheit.py)

- **Funcionalidad**: Publica valores de temperatura en escala Fahrenheit cada segundo
- **Tópico**: `temp_f` (tipo: `std_msgs/Float32`)
- **Rango inicial**: 70°F incrementándose 1°F por cada ciclo
- **Frecuencia**: 1 Hz (1 mensaje por segundo)

**Justificación técnica**:
- Usa `rclpy.Node` para integración nativa con ROS 2
- El timer proporciona periodicidad determinista para simulación de sensor
- Almacena estado (valor actual) para generar datos progresivos

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

### Caso de uso 1: Sistema de Sensores (sensor_pkg)

Este es el primer caso de uso implementado en el proyecto. Demuestra la comunicación básica entre un publicador y un subscriptor.

**Paso 1: Abre la Terminal 1**
```bash
# Asegúrate de estar en el workspace
cd ~/MicroRobotica_ws
source install/setup.bash

# Ejecuta el nodo publicador
ros2 run sensor_pkg pub_temp
```

**Salida esperada en Terminal 1**:
```
temperatura0
temperatura1
temperatura2
temperatura3
...
```

**Paso 2: Abre una Terminal 2 (nueva)**
```bash
# En la terminal nueva
cd ~/MicroRobotica_ws
source install/setup.bash

# Ejecuta el nodo subscriptor
ros2 run sensor_pkg subs_temp
```

**Salida esperada en Terminal 2**:
```
temperatura0
temperatura1
temperatura2
...
```

---

### Caso de uso 2: Sistema de Conversión de Temperaturas (conv_pkg)

Este caso de uso demuestra un pipeline de procesamiento de datos: publicador → procesador → consumidor.

**Paso 1: Abre Terminal 1**
```bash
cd ~/MicroRobotica_ws
source install/setup.bash

# Ejecuta el nodo publicador de Fahrenheit
ros2 run conv_pkg talker_f
```

**Salida esperada**:
```
[INFO] Enviando: 70.0 °F
[INFO] Enviando: 71.0 °F
[INFO] Enviando: 72.0 °F
...
```

**Paso 2: Abre Terminal 2 (nueva)**
```bash
cd ~/MicroRobotica_ws
source install/setup.bash

# Ejecuta el nodo convertidor (suscriptor de F, publicador de C)
ros2 run conv_pkg converter_c
```

**Salida esperada**:
```
[INFO] Recibido: 70.0°F -> Convertido: 21.11°C
[INFO] Recibido: 71.0°F -> Convertido: 21.67°C
[INFO] Recibido: 72.0°F -> Convertido: 22.22°C
...
```

**Paso 3: Terminal 3 (opcional - Monitoreo)**
```bash
cd ~/MicroRobotica_ws
source install/setup.bash

# Monitorea el tópico de Fahrenheit
ros2 topic echo /temp_f
```

O en otra terminal:
```bash
# Monitorea el tópico de Celsius
ros2 topic echo /temp_c

# Ejecuta el nodo subscriptor
ros2 run sensor_pkg subs_temp
```

**Salida esperada en Terminal 2**:
```
temperatura0
temperatura1
temperatura2
...
```

---

### Caso de uso 2: Sistema de Conversión de Temperaturas (conv_pkg)

Este caso de uso demuestra un pipeline de procesamiento de datos: publicador → procesador → consumidor.

**Paso 1: Abre Terminal 1**
```bash
cd ~/MicroRobotica_ws
source install/setup.bash

# Ejecuta el nodo publicador de Fahrenheit
ros2 run conv_pkg talker_f
```

**Salida esperada**:
```
[INFO] Enviando: 70.0 °F
[INFO] Enviando: 71.0 °F
[INFO] Enviando: 72.0 °F
...
```
** | Mode** | Modelo de comunicación basado en DDS, menor latencia, mejor soporte para sistemas distribuidos y embebidos |
| **Python/rclpy** | Desarrollo rápido, integración sencilla con ROS 2, suficiente para lógica de aplicación (no crítica en tiempo real) |
| **ament_python** | Sistema de construcción nativo de ROS 2, mejor integración que sistemas anteriores |
| **std_msgs simple** | Eficiencia de ancho de banda, simplicidad en serialización, adecuado para aplicaciones educativas |
| **Colcon build** | Gestión multiconfigurción, soporte para workspace modular, construcción paralela |
| **Ubuntu 24.04 LTS** | Soporte de 5 años, kernels optimizados, compatibilidad de herramientas, estabilidad probada para robótica |
| **Callbacks síncronos** | Determinismo, control de orden de procesamiento, adecuado para lógica secuencial |
| **Sensor_pkg primero** | Enseña conceptos básicos de Pub-Sub antes de introducir procesamiento de datos |
| **Sensor_pkg primero** | Enseña conceptos básicos de Pub-Sub antes de introducir procesamiento de datos
ros2 run conv_pkg converter_c
```

**Salida esperada**:
```
[INFO] Recibido: 70.0°F -> Convertido: 21.11°C
[INFO] Recibido: 71.0°F -> Convertido: 21.67°C
[INFO] Recibido: 72.0°F -> Convertido: 22.22°C
...
```

**Paso 3: Terminal 3 (opcional - Monitoreo)**
```bash
cd ~/MicroRobotica_ws
source install/setup.bash

# Monitorea el tópico de Fahrenheit
ros2 topic echo /temp_f
```

O en otra terminal:
```bash
# Monitorea el tópico de Celsius
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

## Decisiones Técnicas Justificadas

| Decisión | Justificación |
|----------|--------------|
| **ROS 2 sobre ROS 1** | Mejor modelo de comunicación con DDS, menor latencia, soporte para sistemas embebidos |
| **Python/rclpy** | Desarrollo rápido, integración sencilla con ROS 2, suficiente para lógica de aplicación (no crítica en tiempo real) |
| **ament_python** | Sistema de construcción nativo de ROS 2, mejor integración que catkin |
| **std_msgs simple** | Eficiencia de ancho de banda, simplicidad en serialización, adecuado para aplicaciones educativas |
| **Colcon build** | Gestión multiconfigurción, soporte para workspace modular, construcción paralela |
| **Ubuntu 24.04 LTS** | Soporte de 5 años, kernels optimizados, compatibilidad de herramientas, estabilidad probada |
| **Callbacks síncronos** | Determinismo, control de orden de procesamiento, adecuado para lógica secuencial |

## Extensiones Posibles

1. **Persistencia de datos**: Integrar grabadora ROS 2 (rosbag2) para análisis posterior
2. **Visualización**: Usar `rqt` para dashboards en tiempo real
3. **Control de parámetros**: Implementar `rclpy.parameter` para ajuste dinámico
4. **Testing**: Expandir con pruebas unitarias usando pytest y `launch_ros`
5. **Paralelización**: Implementar nodos con `MultiThreadedExecutor` para operaciones complejas
6. **Hardware real**: Adaptar para sensores físicos (DHT11, termopares, etc.)


