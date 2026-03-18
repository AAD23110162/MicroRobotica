# Micro Robotica: Examen 1er parcial
# Ejercicio 2: Modelo Dinamico de un Robot Cartesiano de 3 GDL (PPP)
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

# --- 1. Modelo dinámico del Robot Cartesiano 3GDL (PPP) ---
def robot_cartesiano(t, x_estado):
    # Vector de estado: 
    # x_estado[0]=x, x_estado[1]=y, x_estado[2]=z (Posiciones en metros)
    # x_estado[3]=xp, x_estado[4]=yp, x_estado[5]=zp (Velocidades en m/s)
    x, y, z = x_estado[0], x_estado[1], x_estado[2]
    xp, yp, zp = x_estado[3], x_estado[4], x_estado[5]
    
    # Parámetros físicos asumidos (Robot industrial mediano)
    g = 9.81      # Gravedad (m/s^2)
    
    # Masas equivalentes de cada eje (kg)
    Mx = 150.0    # El motor X mueve toda la estructura (puente + carro Y + eje Z)
    My = 60.0     # El motor Y mueve el carro y el eje Z
    Mz = 15.0     # El motor Z solo mueve la herramienta/husillo
    
    # Coeficientes de fricción viscosa en las guías lineales (N*s/m)
    bx = 80.0
    by = 40.0
    bz = 20.0
    
    # Fuerzas aplicadas por los motores (Fuerza en Newtons)
    # Eje X: Fuerza senoidal para que oscile de izquierda a derecha
    Fx = 400.0 * np.sin(1.5 * t)
    
    # Eje Y: Fuerza constante que se enciende suavemente
    Fy = 150.0 * (1 - np.exp(-1.0 * t))
    
    # Eje Z: Compensación exacta de gravedad + fuerza extra para subir y bajar
    # Si Fz fuera 0, el eje se caería por su propio peso.
    fuerza_gravedad_z = Mz * g  # ~147.15 N necesarios solo para sostenerlo
    Fz = fuerza_gravedad_z + 50.0 * np.sin(2.0 * t)
    
    # Ecuaciones de movimiento (aceleraciones lineales)
    xpp = (Fx - bx * xp) / Mx
    ypp = (Fy - by * yp) / My
    zpp = (Fz - Mz * g - bz * zp) / Mz  # Aquí restamos la gravedad
    
    # Vector de salida [velocidad_x, vel_y, vel_z, acel_x, acel_y, acel_z]
    return [xp, yp, zp, xpp, ypp, zpp]

# --- 2. Simulación Numérica ---
# Condiciones iniciales: [x(0), y(0), z(0), xp(0), yp(0), zp(0)] = reposo en el origen
estado_inicial = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Intervalo de tiempo: 0 a 10 segundos
t_span = (0, 10)
t_eval = np.linspace(t_span[0], t_span[1], 1000)

# Resolver la Ecuación Diferencial Ordinaria
solucion = solve_ivp(robot_cartesiano, t_span, estado_inicial, t_eval=t_eval, method='RK45')

if not solucion.success:
    raise RuntimeError(f"La simulación del Cartesiano falló: {solucion.message}")

# Extraer resultados
tiempo = solucion.t
pos_x = solucion.y[0]
pos_y = solucion.y[1]
pos_z = solucion.y[2]
vel_x = solucion.y[3]
vel_y = solucion.y[4]
vel_z = solucion.y[5]

# --- 3. Generación de Gráficas ---
configurar_estilo_grafica()
fig, ejes = plt.subplots(2, 1, figsize=(11, 8), sharex=True, constrained_layout=True)

# Posiciones lineales
ejes[0].plot(tiempo, pos_x, label='X (Estructura base)', color='#2563eb', linewidth=2)
ejes[0].plot(tiempo, pos_y, label='Y (Carro transversal)', color='#16a34a', linewidth=2)
ejes[0].plot(tiempo, pos_z, label='Z (Herramienta vertical)', color='#db2777', linewidth=2)
ejes[0].axhline(0.0, color='#64748b', linewidth=1, alpha=0.8)
ejes[0].set_title('Posición Lineal de los Ejes Cartesianos')
ejes[0].set_ylabel('Posición (metros)')
ejes[0].legend(loc='upper left')

# Velocidades lineales
ejes[1].plot(tiempo, vel_x, label='Velocidad X', color='#3b82f6', linestyle='--', linewidth=1.5)
ejes[1].plot(tiempo, vel_y, label='Velocidad Y', color='#22c55e', linestyle='--', linewidth=1.5)
ejes[1].plot(tiempo, vel_z, label='Velocidad Z', color='#f43f5e', linestyle='--', linewidth=1.5)
ejes[1].axhline(0.0, color='#64748b', linewidth=1, alpha=0.8)
ejes[1].set_title('Velocidad Lineal de los Ejes Cartesianos')
ejes[1].set_xlabel('Tiempo (s)')
ejes[1].set_ylabel('Velocidad (m/s)')
ejes[1].legend(loc='lower left')

fig.suptitle('Dinámica de un Robot Cartesiano de 3 GDL (PPP)', fontsize=14, fontweight='bold')
agregar_pie_figura(fig)

# --- Guardado Automático ---
directorio_script = os.path.dirname(os.path.abspath(__file__))
directorio_imagenes = os.path.join(directorio_script, '..', 'Graficas')
os.makedirs(directorio_imagenes, exist_ok=True)

ruta_archivo = os.path.join(directorio_imagenes, 'G002_Cartesiano.png')
fig.savefig(ruta_archivo, dpi=300, bbox_inches='tight')
plt.close(fig)

print(f"¡Éxito! Simulación del robot cartesiano completada y guardada en:\n{ruta_archivo}")