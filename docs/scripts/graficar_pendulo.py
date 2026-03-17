import os
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# --- 1. Parámetros del péndulo (Exactos a la imagen) ---
m = 5.0        # masa
lc = 0.01      # centro de masa
g = 9.81       # constante de aceleración gravitacional
b = 0.17       # coeficiente de fricción viscosa
fc = 0.45      # coeficiente de fricción de Coulomb
Ir = 0.16      # momento de inercia del rotor

# --- 2. Modelo dinámico (Ecuación Diferencial) ---
def pendulo_robot(t, x):
    # vector de estados
    q = x[0]   # posición articular
    qp = x[1]  # velocidad articular
    
    # par aplicado
    tau = 1.5 * np.sin(t) 
    
    # aceleración articular del péndulo (Ecuación línea 15)
    qpp = (tau - b * qp - fc * np.tanh(100000 * qp) - m * g * lc * np.sin(q)) / Ir
    
    # vector de salida
    return [qp, qpp]

# --- 3. Simulación ---
# Condiciones iniciales [q(0)=0, qp(0)=0]
x0 = [0.0, 0.0]

# Intervalo de tiempo: de 0 a 10 segundos
t_span = (0, 10)
t_eval = np.linspace(t_span[0], t_span[1], 1000)

# Resolver la ecuación diferencial
solucion = solve_ivp(pendulo_robot, t_span, x0, t_eval=t_eval, method='RK45')

# Extraer resultados (asumo que se requiere en grados, si la gráfica original era en grados)
tiempo = solucion.t
q_grados = np.degrees(solucion.y[0])
qp_grados = np.degrees(solucion.y[1])

# --- 4. Generación y guardado de la gráfica ---
plt.figure(figsize=(8, 5))

# Graficar
plt.plot(tiempo, q_grados, label='q (grados)', color='black', linewidth=1.2)
plt.plot(tiempo, qp_grados, label='qp (grados/seg)', color='gray', linestyle='--', linewidth=1.2)

# Configuración visual
plt.title('Respuesta del péndulo-robot')
plt.xlabel('Tiempo (s)')
plt.ylabel('Amplitud')
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend(loc='upper right')

# --- Guardado Automático en docs/images/ ---
directorio_script = os.path.dirname(os.path.abspath(__file__))
directorio_imagenes = os.path.join(directorio_script, '..', 'images')
os.makedirs(directorio_imagenes, exist_ok=True)

ruta_archivo = os.path.join(directorio_imagenes, 'respuesta_pendulo.png')
plt.savefig(ruta_archivo, dpi=300, bbox_inches='tight')

print(f"¡Éxito! Simulación completada y guardada en:\n{ruta_archivo}")