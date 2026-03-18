# Micro Robotica: Examen 1er parcial
# Ejercicio 1: Modelo Dinamico de una Centrífuga Industrial (1 GDL)
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

# --- 1. Modelo dinámico de la Centrífuga (1 GDL) ---
def modelo_centrifuga(t, x):
    # Vector de estado: x[0] = posición angular (q), x[1] = velocidad angular (qp)
    q = x[0]
    qp = x[1]
    
    # Parámetros físicos
    m = 10.0      # Masa en kg
    r = 0.5       # Radio en metros
    I = m * r**2  # Momento de inercia (kg*m^2)
    b = 0.5       # Coeficiente de fricción viscosa
    
    # Par aplicado (tau) - Torque que enciende suavemente hasta 15 Nm
    tau = 15.0 * (1 - np.exp(-2.0 * t))
    
    # Ecuación diferencial: q_dos_puntos = (tau - friccion) / Inercia
    qpp = (tau - b * qp) / I
    
    # Vector de salida [velocidad, aceleración]
    return [qp, qpp]

# --- 2. Simulación Numérica ---
# Condiciones iniciales [q(0), qp(0)] = reposo absoluto
x0 = [0.0, 0.0]

# Intervalo de tiempo: 0 a 20 segundos (para ver cómo alcanza la velocidad terminal)
t_span = (0, 20)
t_eval = np.linspace(t_span[0], t_span[1], 1000)

# Resolver la Ecuación Diferencial Ordinaria (RK45)
solucion = solve_ivp(modelo_centrifuga, t_span, x0, t_eval=t_eval, method='RK45')

if not solucion.success:
    raise RuntimeError(f"La simulación de la centrífuga falló: {solucion.message}")

# Extraer resultados
tiempo = solucion.t
# Convertimos de radianes a revoluciones (vueltas) para que sea más intuitivo
q_vueltas = solucion.y[0] / (2 * np.pi)
# Convertimos velocidad a RPM (Revoluciones Por Minuto)
qp_rpm = solucion.y[1] * (60 / (2 * np.pi))

# --- 3. Generación de Gráficas ---
configurar_estilo_grafica()
fig, ejes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)

# Posición articular (Vueltas)
ejes[0].plot(tiempo, q_vueltas, label='q (Posición angular)', color='#10b981', linewidth=2.0)
ejes[0].axhline(0.0, color='#64748b', linewidth=0.9, alpha=0.8)
ejes[0].set_title('Posición de la Centrífuga')
ejes[0].set_ylabel('Posición (Revoluciones)')
ejes[0].legend(loc='upper left')

# Velocidad articular (RPM)
ejes[1].plot(tiempo, qp_rpm, label='qp (Velocidad angular)', color='#8b5cf6', linewidth=2.0)
ejes[1].axhline(0.0, color='#64748b', linewidth=0.9, alpha=0.8)
# Línea asintótica teórica de velocidad terminal (cuando tau = fricción)
rpm_terminal = (15.0 / 0.5) * (60 / (2 * np.pi))
ejes[1].axhline(rpm_terminal, color='#ef4444', linestyle='--', linewidth=1.5, label='Velocidad Terminal Teórica', alpha=0.7)

ejes[1].set_title('Velocidad de la Centrífuga')
ejes[1].set_xlabel('Tiempo (s)')
ejes[1].set_ylabel('Velocidad (RPM)')
ejes[1].legend(loc='lower right')

fig.suptitle('Dinámica de una Centrífuga Industrial (1 GDL)', fontsize=14, fontweight='bold')

# --- Guardado Automático ---
directorio_script = os.path.dirname(os.path.abspath(__file__))
directorio_imagenes = os.path.join(directorio_script, '..', 'Graficas')
os.makedirs(directorio_imagenes, exist_ok=True)

ruta_archivo = os.path.join(directorio_imagenes, 'G001_Centrifuga.png')
fig.savefig(ruta_archivo, dpi=300, bbox_inches='tight')
plt.close(fig)

print(f"¡Éxito! Simulación de la centrífuga completada y guardada en:\n{ruta_archivo}")