# Micro Robotica: Examen 1er parcial
# Ejercicio 4: Modelo Dinamico de un Robot de 3 GDL (RRR)
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


def agregar_pie_figura(fig):
    """Agrega datos de identificacion en la parte inferior de la figura."""
    pie_texto = "Alumno: Alejandro Aguirre Díaz   Registro: 23110162    Examen 1er Parcial de Micro Robotica    Grupo:7E"
    fig.text(0.5, -0.05, pie_texto, ha='center', va='bottom', fontsize=12, color='#334155')

# --- 1. Modelo dinámico del robot 3GDL (Planar RRR) ---
def robot_3gdl(t, x):
    # Vector de estado
    # x[0:3] = q1, q2, q3 (Posiciones)
    # x[3:6] = qp1, qp2, qp3 (Velocidades)
    q1, q2, q3 = x[0], x[1], x[2]
    qp1, qp2, qp3 = x[3], x[4], x[5]
    
    # Parámetros físicos (Masas concentradas al final de cada eslabón)
    m1, m2, m3 = 2.0, 1.5, 1.0  # Masas (kg)
    l1, l2, l3 = 1.0, 0.8, 0.5  # Longitudes (m)
    g = 9.81                    # Gravedad (m/s^2)
    
    # Pre-cálculo de funciones trigonométricas para optimizar
    c2 = np.cos(q2)
    c3 = np.cos(q3)
    c23 = np.cos(q2 + q3)
    s2 = np.sin(q2)
    s3 = np.sin(q3)
    s23 = np.sin(q2 + q3)
    
    # 1. Matriz de Inercia M(q) [3x3]
    M11 = (m1+m2+m3)*l1**2 + (m2+m3)*l2**2 + m3*l3**2 + 2*(m2+m3)*l1*l2*c2 + 2*m3*l1*l3*c23 + 2*m3*l2*l3*c3
    M12 = (m2+m3)*l2**2 + m3*l3**2 + (m2+m3)*l1*l2*c2 + m3*l1*l3*c23 + 2*m3*l2*l3*c3
    M13 = m3*l3**2 + m3*l1*l3*c23 + m3*l2*l3*c3
    M22 = (m2+m3)*l2**2 + m3*l3**2 + 2*m3*l2*l3*c3
    M23 = m3*l3**2 + m3*l2*l3*c3
    M33 = m3*l3**2
    
    M = np.array([
        [M11, M12, M13],
        [M12, M22, M23], # Matriz simétrica (M21 = M12)
        [M13, M23, M33]  # Matriz simétrica (M31 = M13, M32 = M23)
    ])
    
    # 2. Vector de Coriolis y Fuerza Centrífuga V(q, qp) [3x1]
    # En lugar de la inmensa matriz C, calculamos directamente el vector de fuerza resultante
    v1 = -(m2+m3)*l1*l2*s2*(2*qp1*qp2 + qp2**2) - m3*l1*l3*s23*(2*qp1*qp2 + 2*qp1*qp3 + 2*qp2*qp3 + qp2**2 + qp3**2) - m3*l2*l3*s3*(2*qp1*qp3 + 2*qp2*qp3 + qp3**2)
    v2 = (m2+m3)*l1*l2*s2*qp1**2 + m3*l1*l3*s23*qp1**2 - m3*l2*l3*s3*(2*qp1*qp3 + 2*qp2*qp3 + qp3**2)
    v3 = m3*l1*l3*s23*qp1**2 + m3*l2*l3*s3*(qp1 + qp2)**2
    V = np.array([v1, v2, v3])
    
    # 3. Vector de Gravedad G(q) [3x1]
    # Asumimos que 0 grados es el eje X positivo (horizontal), gravedad en -Y
    g1 = (m1+m2+m3)*g*l1*np.cos(q1) + (m2+m3)*g*l2*np.cos(q1+q2) + m3*g*l3*np.cos(q1+q2+q3)
    g2 = (m2+m3)*g*l2*np.cos(q1+q2) + m3*g*l3*np.cos(q1+q2+q3)
    g3 = m3*g*l3*np.cos(q1+q2+q3)
    G = np.array([g1, g2, g3])
    
    # 4. Fricción y Pares aplicados
    # Aplicamos fricción viscosa en las articulaciones para que el sistema eventualmente se detenga
    fr = np.array([3.0 * qp1, 2.0 * qp2, 1.0 * qp3])
    
    # Sin fuerza en los motores (caída libre bajo gravedad)
    tau = np.array([0.0, 0.0, 0.0])
    
    # 5. Resolver aceleraciones: M * q_dos_puntos = tau - V - G - fr
    vector_fuerzas = tau - V - G - fr
    q_dos_puntos = np.linalg.solve(M, vector_fuerzas)
    
    return [qp1, qp2, qp3, q_dos_puntos[0], q_dos_puntos[1], q_dos_puntos[2]]

# --- 2. Simulación Numérica ---
# Condiciones iniciales: Robot completamente estirado en horizontal (0 grados)
x0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Intervalo de tiempo: 0 a 20 segundos
t_span = (0, 20)
t_eval = np.linspace(t_span[0], t_span[1], 1500)

solucion = solve_ivp(robot_3gdl, t_span, x0, t_eval=t_eval, method='RK45')

if not solucion.success:
    raise RuntimeError(f"La simulación del 3GDL falló: {solucion.message}")

# Extraer resultados en grados
tiempo = solucion.t
q_grados = np.degrees(solucion.y[0:3])
qp_grados = np.degrees(solucion.y[3:6])

# --- 3. Generación de Gráficas ---
configurar_estilo_grafica()
fig, ejes = plt.subplots(2, 1, figsize=(11, 8), sharex=True, constrained_layout=True)

# Posiciones
ejes[0].plot(tiempo, q_grados[0], label='q1 (Hombro)', color='#1d4ed8', linewidth=1.8)
ejes[0].plot(tiempo, q_grados[1], label='q2 (Codo)', color='#dc2626', linewidth=1.8, linestyle='--')
ejes[0].plot(tiempo, q_grados[2], label='q3 (Muñeca)', color='#16a34a', linewidth=1.8, linestyle='-.')
ejes[0].axhline(-90.0, color='#64748b', linewidth=1, label='Reposo vertical (-90°)', alpha=0.8)
ejes[0].set_title('Posición Articular (Caída libre bajo gravedad)')
ejes[0].set_ylabel('Ángulo (grados)')
ejes[0].legend(loc='upper right')

# Velocidades
ejes[1].plot(tiempo, qp_grados[0], label='Velocidad q1', color='#3b82f6', linewidth=1.5)
ejes[1].plot(tiempo, qp_grados[1], label='Velocidad q2', color='#ef4444', linewidth=1.5, linestyle='--')
ejes[1].plot(tiempo, qp_grados[2], label='Velocidad q3', color='#22c55e', linewidth=1.5, linestyle='-.')
ejes[1].axhline(0.0, color='#64748b', linewidth=1, alpha=0.8)
ejes[1].set_title('Velocidad Articular')
ejes[1].set_xlabel('Tiempo (s)')
ejes[1].set_ylabel('Velocidad (grados/s)')
ejes[1].legend(loc='upper right')

fig.suptitle('Dinámica de un Robot Planar de 3 GDL (RRR)', fontsize=14, fontweight='bold')
agregar_pie_figura(fig)

# --- Guardado Automático ---
directorio_script = os.path.dirname(os.path.abspath(__file__))
directorio_imagenes = os.path.join(directorio_script, '..', 'Graficas')
os.makedirs(directorio_imagenes, exist_ok=True)

ruta_archivo = os.path.join(directorio_imagenes, 'G004_RRR.png')
fig.savefig(ruta_archivo, dpi=300, bbox_inches='tight')
plt.close(fig)

print(f"¡Éxito! Simulación del 3GDL completada y guardada en:\n{ruta_archivo}")