import sympy as sp
import sys

def derivar_centrifuga():
    print("\n" + "="*50)
    print(" Iniciando deducción de la Centrífuga (1 GDL)...")
    print("="*50)
    t = sp.Symbol('t')
    q = sp.Function('q')(t)
    dq = sp.diff(q, t)
    I = sp.symbols('I')
    
    # Energías
    K = 0.5 * I * dq**2
    U = 0  # Movimiento horizontal, no hay cambio de energía potencial
    
    # Euler-Lagrange
    M11 = sp.simplify(sp.diff(sp.diff(K, dq), dq))
    G1 = sp.diff(U, q)
    
    print("\n--- MATRIZ DE INERCIA M(q) ---")
    sp.pprint(M11)
    print("\n--- VECTOR DE GRAVEDAD G(q) ---")
    sp.pprint(G1)
    print("\nNota: Al ser 1 GDL rotacional puro, su inercia es constante (I) y no le afecta la gravedad.")

def derivar_cartesiano():
    print("\n" + "="*50)
    print(" Iniciando deducción del Cartesiano 3GDL (PPP)...")
    print("="*50)
    t = sp.Symbol('t')
    x = sp.Function('x')(t)
    y = sp.Function('y')(t)
    z = sp.Function('z')(t)
    dx, dy, dz = sp.diff(x, t), sp.diff(y, t), sp.diff(z, t)
    
    mx, my, mz, g = sp.symbols('mx my mz g')
    
    # Energías
    K = 0.5*mx*dx**2 + 0.5*my*dy**2 + 0.5*mz*dz**2
    U = mz*g*z  # Solo el eje Z lucha contra la gravedad
    
    # Gravedad
    G_vec = sp.Matrix([sp.diff(U, x), sp.diff(U, y), sp.diff(U, z)])
    
    # Inercia (Derivada doble de K respecto a las velocidades)
    M = sp.Matrix([
        [sp.diff(sp.diff(K, dx), dx), sp.diff(sp.diff(K, dx), dy), sp.diff(sp.diff(K, dx), dz)],
        [sp.diff(sp.diff(K, dy), dx), sp.diff(sp.diff(K, dy), dy), sp.diff(sp.diff(K, dy), dz)],
        [sp.diff(sp.diff(K, dz), dx), sp.diff(sp.diff(K, dz), dy), sp.diff(sp.diff(K, dz), dz)]
    ])
    
    print("\n--- MATRIZ DE INERCIA M(q) ---")
    sp.pprint(M)
    print("\n--- VECTOR DE GRAVEDAD G(q) ---")
    sp.pprint(G_vec)
    print("\nNota: Ejes totalmente desacoplados (matriz diagonal). Solo el eje Z tiene componente gravitacional.")

def derivar_modelo_rr():
    print("\n" + "="*50)
    print(" Iniciando deducción del robot Planar 2GDL (RR)...")
    print("="*50)
    t = sp.Symbol('t')
    q1 = sp.Function('q1')(t)
    q2 = sp.Function('q2')(t)
    dq1, dq2 = sp.diff(q1, t), sp.diff(q2, t)
    
    m1, m2, I1, I2, l1, lc1, lc2, g = sp.symbols('m1 m2 I1 I2 l1 lc1 lc2 g')
    
    # Cinemática Directa
    y1 = lc1 * sp.sin(q1)
    x1 = lc1 * sp.cos(q1)
    x2 = l1 * sp.cos(q1) + lc2 * sp.cos(q1 + q2)
    y2 = l1 * sp.sin(q1) + lc2 * sp.sin(q1 + q2)
    
    # Velocidades al cuadrado
    v1_sq = sp.simplify(sp.diff(x1, t)**2 + sp.diff(y1, t)**2)
    v2_sq = sp.simplify(sp.diff(x2, t)**2 + sp.diff(y2, t)**2)
    
    # Energías
    K = 0.5*m1*v1_sq + 0.5*I1*dq1**2 + 0.5*m2*v2_sq + 0.5*I2*(dq1 + dq2)**2
    U = m1*g*y1 + m2*g*y2
    
    # Gravedad
    G_vec = sp.Matrix([sp.simplify(sp.diff(U, q1)), sp.simplify(sp.diff(U, q2))])
    
    # Inercia
    M11 = sp.simplify(sp.diff(sp.diff(K, dq1), dq1))
    M12 = sp.simplify(sp.diff(sp.diff(K, dq1), dq2))
    M22 = sp.simplify(sp.diff(sp.diff(K, dq2), dq2))
    M = sp.Matrix([[M11, M12], [M12, M22]])
    
    print("\n--- MATRIZ DE INERCIA M(q) ---")
    sp.pprint(M)
    print("\n--- VECTOR DE GRAVEDAD G(q) ---")
    sp.pprint(G_vec)

def derivar_modelo_rrr():
    print("\n" + "="*50)
    print(" Iniciando deducción simbólica del robot 3GDL (RRR)...")
    print("="*50)
    
    t = sp.Symbol('t')
    q1 = sp.Function('q1')(t)
    q2 = sp.Function('q2')(t)
    q3 = sp.Function('q3')(t)
    dq1, dq2, dq3 = sp.diff(q1, t), sp.diff(q2, t), sp.diff(q3, t)
    
    m1, m2, m3 = sp.symbols('m1 m2 m3')
    I1, I2, I3 = sp.symbols('I1 I2 I3')
    l1, l2, l3 = sp.symbols('l1 l2 l3')
    lc1, lc2, lc3 = sp.symbols('lc1 lc2 lc3')
    g = sp.symbols('g')
    
    # Cinemática Directa (Centros de Masa)
    x1 = lc1 * sp.cos(q1)
    y1 = lc1 * sp.sin(q1)
    
    x2 = l1 * sp.cos(q1) + lc2 * sp.cos(q1 + q2)
    y2 = l1 * sp.sin(q1) + lc2 * sp.sin(q1 + q2)
    
    x3 = l1 * sp.cos(q1) + l2 * sp.cos(q1 + q2) + lc3 * sp.cos(q1 + q2 + q3)
    y3 = l1 * sp.sin(q1) + l2 * sp.sin(q1 + q2) + lc3 * sp.sin(q1 + q2 + q3)
    
    # Velocidades lineales al cuadrado
    print("Calculando velocidades de los 3 eslabones (esto tomará unos segundos)...")
    v1_sq = sp.simplify(sp.diff(x1, t)**2 + sp.diff(y1, t)**2)
    v2_sq = sp.simplify(sp.diff(x2, t)**2 + sp.diff(y2, t)**2)
    v3_sq = sp.simplify(sp.diff(x3, t)**2 + sp.diff(y3, t)**2)
    
    w1 = dq1
    w2 = dq1 + dq2
    w3 = dq1 + dq2 + dq3
    
    # Energías
    K = 0.5*m1*v1_sq + 0.5*I1*w1**2 + 0.5*m2*v2_sq + 0.5*I2*w2**2 + 0.5*m3*v3_sq + 0.5*I3*w3**2
    U = m1*g*y1 + m2*g*y2 + m3*g*y3
    
    print("\n--- VECTOR DE GRAVEDAD G(q) ---")
    G1 = sp.simplify(sp.diff(U, q1))
    G2 = sp.simplify(sp.diff(U, q2))
    G3 = sp.simplify(sp.diff(U, q3))
    
    print("G_1:")
    sp.pprint(G1)
    print("\nG_2:")
    sp.pprint(G2)
    print("\nG_3:")
    sp.pprint(G3)
    
    print("\n--- MATRIZ DE INERCIA M(q) ---")
    dqs = [dq1, dq2, dq3]
    M11 = sp.simplify(sp.diff(sp.diff(K, dq1), dq1))
    print("Componente M_11 (Inercia sentida por el hombro):")
    sp.pprint(M11)
    print("\n(Nota: Solo se imprime M_11 porque la matriz completa 3x3 es demasiado extensa para la consola).")

def mostrar_menu():
    while True:
        print("\n" + "#"*60)
        print(" SISTEMA DE DEDUCCIÓN DE MODELOS DINÁMICOS (Euler-Lagrange)")
        print("#"*60)
        print("1. Centrífuga (1 GDL)")
        print("2. Robot Planar (2 GDL - RR)")
        print("3. Robot Cartesiano (3 GDL - PPP)")
        print("4. Robot Planar (3 GDL - RRR)")
        print("5. Salir")
        print("-" * 60)
        
        opcion = input("Seleccione el modelo a deducir (1-5): ")
        
        if opcion == '1':
            derivar_centrifuga()
        elif opcion == '2':
            derivar_modelo_rr()
        elif opcion == '3':
            derivar_cartesiano()
        elif opcion == '4':
            derivar_modelo_rrr()
        elif opcion == '5':
            print("Saliendo del programa...")
            sys.exit(0)
        else:
            print("Opción no válida. Por favor, introduzca un número del 1 al 5.")
            
        input("\nPresione ENTER para volver al menú principal...")

if __name__ == "__main__":
    mostrar_menu()