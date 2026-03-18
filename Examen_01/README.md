# Examen del Primer Parcial - Modelado Dinamico de Robots

## Datos del estudiante

|  |  |
|---|---|
| Alumno | Alejandro Aguirre Díaz |
| Registro | 23110162 |
| Grupo | 7E |
| Fecha de entrega | Miercoles 18 de marzo de 2026 |

Este directorio del repositorio MicroRobotica contiene los 4 ejercicios del examen del primer parcial. En cada ejercicio se realiza el modelado dinamico y la simulacion numerica en Python para obtener sus graficas de comportamiento.

## Descripcion del examen

Realizar y graficar el modelo dinamico de:

1. Centrifuga industrial (1 GDL)
2. Robot cartesiano PPP (3 GDL)
3. Robot planar RR (2 GDL)
4. Robot planar RRR (3 GDL)

## Estructura del examen

| Ejercicio | Robot | Script | Grafica generada |
|---|---|---|---|
| 1 | Centrifuga (1 GDL) | [Scripts/001_Centrifuga.py](Scripts/001_Centrifuga.py) | [Graficas/G001_Centrifuga.png](Graficas/G001_Centrifuga.png) |
| 2 | Cartesiano PPP (3 GDL) | [Scripts/002_Cartesiano.py](Scripts/002_Cartesiano.py) | [Graficas/G002_Cartesiano.png](Graficas/G002_Cartesiano.png) |
| 3 | Robot RR (2 GDL) | [Scripts/003_RR.py](Scripts/003_RR.py) | [Graficas/G003_RR.png](Graficas/G003_RR.png) |
| 4 | Robot RRR (3 GDL) | [Scripts/004_RRR.py](Scripts/004_RRR.py) | [Graficas/G004_RRR.png](Graficas/G004_RRR.png) |

## Dependencias

Los scripts usan:

- Python 3
- numpy
- scipy
- matplotlib

Instalacion recomendada:

```bash
python3 -m pip install numpy scipy matplotlib
```

## Ejecucion

Desde esta carpeta:

```bash
cd Scripts
python3 001_Centrifuga.py
python3 002_Cartesiano.py
python3 003_RR.py
python3 004_RRR.py
```

Cada script resuelve su sistema con `solve_ivp(..., method='RK45')` y guarda automaticamente su grafica en `../Graficas/`.

---

## Marco matematico comun

La forma general del modelo dinamico en robots se expresa como:

$$
M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + G(q) + F_r(\dot{q}) = \tau
$$

donde:

- $q$: coordenadas generalizadas (angulares o lineales)
- $M(q)$: matriz de inercia
- $C(q,\dot{q})\dot{q}$: efectos de Coriolis/centrifugos
- $G(q)$: terminos de gravedad
- $F_r(\dot{q})$: friccion
- $\tau$: entradas (pares o fuerzas)

Para integrar numericamente, cada modelo se pasa a primer orden con el estado:

$$
x = \begin{bmatrix} q \\ \dot{q} \end{bmatrix},
\qquad
\dot{x} = f(t,x)
$$

---

## Ejercicio 1 - Centrifuga industrial (1 GDL)

Referencia:

- Codigo: [Scripts/001_Centrifuga.py](Scripts/001_Centrifuga.py)
- Grafica: [Graficas/G001_Centrifuga.png](Graficas/G001_Centrifuga.png)

### Variables y parametros

- Coordenada: $q$ (angulo)
- Velocidad: $\dot{q}$
- Masa: $m = 10\,kg$
- Radio: $r = 0.5\,m$
- Inercia: $I = mr^2 = 2.5\,kg\,m^2$
- Friccion viscosa: $b = 0.5$
- Par aplicado:

$$
tau(t) = 15\,(1-e^{-2t})\,\text{N m}
$$

### Ecuacion dinamica

$$
I\ddot{q} + b\dot{q} = \tau(t)
$$

Despejando aceleracion:

$$
\ddot{q} = \frac{\tau(t)-b\dot{q}}{I}
$$

### Forma de estado

Con $x_1=q$, $x_2=\dot{q}$:

$$
\dot{x}_1 = x_2
$$

$$
\dot{x}_2 = \frac{15(1-e^{-2t}) - 0.5x_2}{2.5}
$$

### Simulacion

- Condicion inicial: $[q(0),\dot{q}(0)] = [0,0]$
- Tiempo: $0\,s$ a $20\,s$
- Salidas graficadas: posicion (vueltas) y velocidad (RPM)

### Grafica del ejercicio

![Dinamica de la centrifuga industrial](Graficas/G001_Centrifuga.png)

### Observaciones principales

- La posicion angular crece de forma monotona, indicando giro continuo sin invertir sentido.
- La velocidad en RPM sube rapidamente durante el transitorio y luego se acerca a un valor de equilibrio.
- La linea teorica de velocidad terminal coincide con el comportamiento esperado por el balance entre par aplicado y friccion viscosa.

Velocidad terminal teorica (linea punteada en la figura):

$$
\omega_{\infty} = \frac{\tau_{max}}{b} = \frac{15}{0.5} = 30\,rad/s
$$

$$
RPM_{\infty} = 30\frac{60}{2\pi} \approx 286.48\,RPM
$$

---

## Ejercicio 2 - Robot cartesiano PPP (3 GDL)

Referencia:

- Codigo: [Scripts/002_Cartesiano.py](Scripts/002_Cartesiano.py)
- Grafica: [Graficas/G002_Cartesiano.png](Graficas/G002_Cartesiano.png)

### Variables y parametros

- Coordenadas lineales: $q = [x, y, z]^T$
- Velocidades: $\dot{q} = [\dot{x}, \dot{y}, \dot{z}]^T$
- Masas equivalentes:

$$
M_x = 150, \quad M_y = 60, \quad M_z = 15\,kg
$$

- Friccion viscosa:

$$
b_x=80, \quad b_y=40, \quad b_z=20
$$

- Entradas:

$$
F_x(t) = 400\sin(1.5t)
$$

$$
F_y(t) = 150(1-e^{-t})
$$

$$
F_z(t) = M_zg + 50\sin(2t)
$$

### Ecuaciones dinamicas

$$
M_x\ddot{x} + b_x\dot{x} = F_x(t)
$$

$$
M_y\ddot{y} + b_y\dot{y} = F_y(t)
$$

$$
M_z\ddot{z} + b_z\dot{z} + M_zg = F_z(t)
$$

En el codigo, el eje $z$ resta explicitamente la gravedad:

$$
\ddot{z} = \frac{F_z - M_zg - b_z\dot{z}}{M_z}
$$

### Forma de estado

Con $x_e=[x,y,z,\dot{x},\dot{y},\dot{z}]^T$:

$$
\dot{x}_e = [\dot{x},\dot{y},\dot{z},\ddot{x},\ddot{y},\ddot{z}]^T
$$

### Simulacion

- Condicion inicial: reposo en el origen
- Tiempo: $0\,s$ a $10\,s$
- Salidas: posicion y velocidad de los tres ejes

### Grafica del ejercicio

![Dinamica del robot cartesiano PPP](Graficas/G002_Cartesiano.png)

### Observaciones principales

- El eje X muestra oscilaciones alrededor del origen por la fuerza sinusoidal aplicada y la amortiguacion viscosa.
- El eje Y presenta una respuesta de crecimiento transitorio y estabilizacion progresiva, consistente con una excitacion que se activa suavemente.
- El eje Z oscila en torno a una condicion de equilibrio vertical, ya que la gravedad esta compensada y solo queda la componente sinusoidal adicional.

---

## Ejercicio 3 - Robot RR (2 GDL)

Referencia:

- Codigo: [Scripts/003_RR.py](Scripts/003_RR.py)
- Grafica: [Graficas/G003_RR.png](Graficas/G003_RR.png)

### Variables de estado

$$
q = [q_1, q_2]^T, \qquad \dot{q} = [\dot{q}_1, \dot{q}_2]^T
$$

### Modelo dinamico implementado

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) + f_r(\dot{q}) = \tau(t)
$$

con:

$$
M(q)=
\begin{bmatrix}
3.117 + 0.2\cos(q_2) & 0.108 + 0.1\cos(q_2) \\
0.108 + 0.1\cos(q_2) & 0.108
\end{bmatrix}
$$

$$
C(q,\dot{q})=
\begin{bmatrix}
-0.2\sin(q_2)\dot{q}_2 & -0.1\sin(q_2)\dot{q}_2 \\
0.1\sin(q_2)\dot{q}_1 & 0
\end{bmatrix}
$$

$$
g(q)=
\begin{bmatrix}
39.3\sin(q_1)+1.95\sin(q_1+q_2) \\
1.95\sin(q_1+q_2)
\end{bmatrix}
$$

$$
f_r(\dot{q})=
\begin{bmatrix}
1.86\dot{q}_1 + 1.93\tanh(10^5\dot{q}_1) \\
0.16\dot{q}_2 + 0.3\tanh(10^5\dot{q}_2)
\end{bmatrix}
$$

Los pares de entrada son excitaciones mixtas (rampa exponencial + senoidales):

$$
tau_1(t) = (1-e^{-0.8t})32 + 56\sin(16t+0.1) + 12\sin(20t+0.15)
$$

$$
tau_2(t) = (1-e^{-1.8t})1.2 + 8\sin(26t+0.08) + 2\sin(12t+0.34)
$$

En simulacion se calcula:

$$
\ddot{q} = M^{-1}\left(\tau - C\dot{q} - g - f_r\right)
$$

(implementado con `np.linalg.solve(M, ...)` para mayor estabilidad numerica).

### Simulacion

- Condicion inicial: $[q_1,q_2,\dot{q}_1,\dot{q}_2]=[0,0,0,0]$
- Tiempo: $0\,s$ a $10\,s$
- Salidas: posiciones y velocidades en grados y grados/s

### Grafica del ejercicio

![Dinamica del robot RR de 2 GDL](Graficas/G003_RR.png)

### Observaciones principales

- Las posiciones articulares muestran oscilaciones no lineales con contenido de alta frecuencia debido a la combinacion de entradas sinusoidales.
- Las velocidades presentan picos transitorios y variaciones acopladas entre articulaciones, reflejando la matriz de inercia dependiente de la configuracion.
- La friccion viscosa y Coulomb suavizada limita el crecimiento indefinido de la respuesta y mantiene la simulacion numericamente estable.

---

## Ejercicio 4 - Robot RRR planar (3 GDL)

Referencia:

- Codigo: [Scripts/004_RRR.py](Scripts/004_RRR.py)
- Grafica: [Graficas/G004_RRR.png](Graficas/G004_RRR.png)

### Variables y parametros

$$
q=[q_1,q_2,q_3]^T, \qquad \dot{q}=[\dot{q}_1,\dot{q}_2,\dot{q}_3]^T
$$

$$
m_1=2.0,\ m_2=1.5,\ m_3=1.0\,kg
$$

$$
l_1=1.0,\ l_2=0.8,\ l_3=0.5\,m
$$

$$
g=9.81\,m/s^2
$$

### Modelo dinamico implementado

El script usa la forma:

$$
M(q)\ddot{q} + V(q,\dot{q}) + G(q) + f_r(\dot{q}) = \tau
$$

donde:

- $M(q)$ se arma de forma explicita con terminos $M_{11},\dots,M_{33}$
- $V(q,\dot{q})=[v_1,v_2,v_3]^T$ concentra terminos Coriolis/centrifugos
- $G(q)=[g_1,g_2,g_3]^T$ contiene gravedad
- $f_r=[3\dot{q}_1, 2\dot{q}_2, \dot{q}_3]^T$
- $\tau=[0,0,0]^T$ (caida libre con friccion)

Componentes de gravedad usadas en el codigo:

$$
g_1=(m_1+m_2+m_3)gl_1\cos q_1 + (m_2+m_3)gl_2\cos(q_1+q_2) + m_3gl_3\cos(q_1+q_2+q_3)
$$

$$
g_2=(m_2+m_3)gl_2\cos(q_1+q_2) + m_3gl_3\cos(q_1+q_2+q_3)
$$

$$
g_3=m_3gl_3\cos(q_1+q_2+q_3)
$$

La aceleracion se obtiene resolviendo:

$$
\ddot{q}=M^{-1}(\tau - V - G - f_r)
$$

### Simulacion

- Condicion inicial: robot estirado horizontalmente

$$
[q_1,q_2,q_3,\dot{q}_1,\dot{q}_2,\dot{q}_3] = [0,0,0,0,0,0]
$$

- Tiempo: $0\,s$ a $20\,s$
- Salidas: posiciones y velocidades articulares (grados y grados/s)

### Grafica del ejercicio

![Dinamica del robot RRR planar de 3 GDL](Graficas/G004_RRR.png)

### Observaciones principales

- El robot inicia en una posicion de alta energia potencial y evoluciona bajo gravedad hacia configuraciones de menor energia.
- Las velocidades articulares aumentan en el transitorio inicial y luego decrecen por el efecto disipativo de la friccion viscosa.
- Se aprecia fuerte acoplamiento dinamico entre articulaciones: el movimiento de un eslabon modifica la respuesta de los demas durante toda la caida libre.

---

## Interpretacion global de resultados

1. Centrifuga: respuesta transitoria con saturacion de velocidad por equilibrio entre par y friccion.
2. Cartesiano PPP: comportamiento desacoplado por eje con compensacion de gravedad en $z$.
3. RR: dinamica acoplada no lineal con inercia dependiente de configuracion, gravedad y friccion mixta.
4. RRR: caida libre no lineal acoplada con disipacion por friccion viscosa.

Este documento organiza de forma uniforme el desarrollo matematico, la simulacion y la interpretacion de resultados para los cuatro sistemas roboticos solicitados en el examen.


