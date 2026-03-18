# Micro Robotica: Examen 1er parcial
# Ejercicio 3: Modelo Dinamico de un Robot de 2 GDL (RR)
# Autor: Alejandro Aguirre Díaz
# Fecha: Miercoles 18 de marzo del 2026

import os
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def configurar_estilo_grafica():
    """Configura un estilo legible y consistente para todas las figuras."""
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'figure.facecolor': 'white',
        'axes.facecolor': '#f8fafc',
        'axes.edgecolor': '#334155',
        'axes.titlesize': 12,
        'axes.labelsize': 11,
        'grid.color': '#94a3b8',
        'grid.linestyle': ':',
        'grid.alpha': 0.5,
        'legend.frameon': True,
        'legend.framealpha': 0.9,
    })

# --- 1. Modelo dinámico del robot 2GDL (Traducción exacta) ---
def robot_2gdl(t, x):
    # Vector de posición y velocidad articular
    # x[0]=q1, x[1]=q2, x[2]=qp1, x[3]=qp2
    q1, q2 = x[0], x[1]
    qp1, qp2 = x[2], x[3]
    
    # Matriz de Inercia (M)
    M = np.array([
        [3.117 + 0.2 * np.cos(q2), 0.108 + 0.1 * np.cos(q2)],
        [0.108 + 0.1 * np.cos(q2), 0.108]
    ])
    
    # Matriz de fuerzas centrípetas y de Coriolis (C)
    C = np.array([
        [-0.2 * np.sin(q2) * qp2, -0.1 * np.sin(q2) * qp2],
        [ 0.1 * np.sin(q2) * qp1,  0.0]
    ])
    
    # Vector de pares gravitacionales (par_grav)
    par_grav = np.array([
        39.3 * np.sin(q1) + 1.95 * np.sin(q1 + q2),
        1.95 * np.sin(q1 + q2)
    ])
    
    # Vector de pares de fricción viscosa y Coulomb (fr)
    # Usamos np.tanh para suavizar la función signo y asegurar la estabilidad del solver numérico
    fr = np.array([
        1.86 * qp1 + 1.93 * np.tanh(100000 * qp1),
        0.16 * qp2 + 0.3 * np.tanh(100000 * qp2)
    ])
    
    # Par aplicado (tau)
    tau = np.array([
        (1 - np.exp(-0.8 * t)) * 32.0 + 56 * np.sin(16 * t + 0.1) + 12 * np.sin(20 * t + 0.15),
        (1 - np.exp(-1.8 * t)) * 1.2  + 8 * np.sin(26 * t + 0.08) + 2 * np.sin(12 * t + 0.34)
    ])
    
    # Vector de aceleración articular (q2p)
    # En Python, en lugar de inv(M)*rhs, usamos linalg.solve(M, rhs) por estabilidad
    vector_fuerzas = tau - (C @ np.array([qp1, qp2])) - par_grav - fr
    q2p = np.linalg.solve(M, vector_fuerzas)
    
    # Vector de salida [velocidad_1, velocidad_2, aceleracion_1, aceleracion_2]
    return [qp1, qp2, q2p[0], q2p[1]]

# --- 2. Simulación Numérica ---
# Condiciones iniciales [q1(0), q2(0), qp1(0), qp2(0)] = reposo absoluto
x0 = [0.0, 0.0, 0.0, 0.0]

# Intervalo de tiempo: 0 a 10 segundos
t_span = (0, 10)
t_eval = np.linspace(t_span[0], t_span[1], 1000)

# Resolver la Ecuación Diferencial Ordinaria (RK45)
solucion = solve_ivp(robot_2gdl, t_span, x0, t_eval=t_eval, method='RK45')

if not solucion.success:
    raise RuntimeError(f"La simulación del robot 2GDL falló: {solucion.message}")

# Extraer resultados (pasando las posiciones de radianes a grados para facilitar la lectura)
tiempo = solucion.t
q1_grados = np.degrees(solucion.y[0])
q2_grados = np.degrees(solucion.y[1])
qp1_grados = np.degrees(solucion.y[2])
qp2_grados = np.degrees(solucion.y[3])

# --- 3. Generación de Gráficas ---
configurar_estilo_grafica()
fig, ejes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)

# Posiciones articulares
ejes[0].plot(tiempo, q1_grados, label='q1 (eslabón 1)', color='#1d4ed8', linewidth=1.8)
ejes[0].plot(tiempo, q2_grados, label='q2 (eslabón 2)', color='#dc2626', linestyle='--', linewidth=1.8)
ejes[0].axhline(0.0, color='#64748b', linewidth=0.9, alpha=0.8)
ejes[0].set_title('Posición articular')
ejes[0].set_ylabel('Ángulo (grados)')
ejes[0].legend(loc='upper right')

# Velocidades articulares
ejes[1].plot(tiempo, qp1_grados, label='qp1 (eslabón 1)', color='#0284c7', linewidth=1.8)
ejes[1].plot(tiempo, qp2_grados, label='qp2 (eslabón 2)', color='#ea580c', linestyle='--', linewidth=1.8)
ejes[1].axhline(0.0, color='#64748b', linewidth=0.9, alpha=0.8)
ejes[1].set_title('Velocidad articular')
ejes[1].set_xlabel('Tiempo (s)')
ejes[1].set_ylabel('Velocidad (grados/s)')
ejes[1].legend(loc='upper right')

fig.suptitle('Dinámica del robot de transmisión directa de 2GDL', fontsize=14, fontweight='bold')

# --- Guardado Automático ---
directorio_script = os.path.dirname(os.path.abspath(__file__))
directorio_imagenes = os.path.join(directorio_script, '..', 'Graficas')
os.makedirs(directorio_imagenes, exist_ok=True)

ruta_archivo = os.path.join(directorio_imagenes, 'G003_RR.png')
fig.savefig(ruta_archivo, dpi=300, bbox_inches='tight')
plt.close(fig)

print(f"¡Éxito! Simulación 2GDL completada y guardada en:\n{ruta_archivo}")