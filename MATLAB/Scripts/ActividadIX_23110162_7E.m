% Actividad IX: Control Cartesiano
% Alejandro Aguirre Díaz-23110162
% MicroRobotica 7-E
% Modelo Dinamico Cartesiano para trazar un circulo
clear; clc; close all;

% -------------------------------------------------------------------------
% 1. ASIGNACIÓN DE PARÁMETROS DEL ROBOT Y CONTROLADOR
% -------------------------------------------------------------------------
params.m1 = 23.902; params.l1 = 0.45; params.lc1 = 0.091; 
params.I1 = 1.266;  params.b1 = 2.288;
params.m2 = 3.880;  params.l2 = 0.45; params.lc2 = 0.048; 
params.I2 = 0.093;  params.b2 = 0.175; 
params.g  = 9.81;

% Ganancias de la Ley de Control
params.Kp = [3000, 0; 0, 3000]; 
params.Kv = [200, 0; 0, 200];

% -------------------------------------------------------------------------
% 2. GENERACIÓN DE LA TRAYECTORIA (Círculo cerrado)
% -------------------------------------------------------------------------
tiempo_vuelta = 10; % Segundos para dar exactamente una vuelta
vueltas = 1.1;      % Le pedimos 1.2 vueltas para asegurar que cierre el trazo
tiempo_total = tiempo_vuelta * vueltas; 

N_puntos = 10;     % Aumentamos los puntos para una interpolación más suave

t_span = linspace(0, tiempo_total, N_puntos); 

Xc = 0.4; Yc = -0.2; R = 0.15; 
w = 2*pi / tiempo_vuelta; % La velocidad angular debe basarse en 1 vuelta

x_coordenadas = Xc + R * cos(w * t_span);
y_coordenadas = Yc + R * sin(w * t_span);

disp(['Calculando simulación con ', num2str(N_puntos), ' puntos durante ', num2str(tiempo_total), 's...']);

% -------------------------------------------------------------------------
% 3. LLAMADA A LA FUNCIÓN DE MOVIMIENTO
% -------------------------------------------------------------------------
[t_sim, estado] = move(x_coordenadas, y_coordenadas, tiempo_total, params);

% -------------------------------------------------------------------------
% 4. EXTRACCIÓN DE DATOS Y CINEMÁTICA PARA GRÁFICAS
% -------------------------------------------------------------------------
q1  = estado(:, 1); 
q2  = estado(:, 2);
qp1 = estado(:, 3); 
qp2 = estado(:, 4); 

% Posición real cartesiana
X_real = params.l1.*sin(q1) + params.l2.*sin(q1+q2);
Y_real = -params.l1.*cos(q1) - params.l2.*cos(q1+q2);

% -------------------------------------------------------------------------
% 5. REALIZACIÓN DE GRÁFICAS
% -------------------------------------------------------------------------

% --- FIGURA 1: Espacio Cartesiano (Trayectoria) ---
figure('Name', 'Modelo Dinámico Cartesiano - Trayectoria', 'Color', 'w');
plot(x_coordenadas, y_coordenadas, 'r--', 'LineWidth', 2); hold on;
plot(X_real, Y_real, 'b.-', 'MarkerSize', 8, 'LineWidth', 1); 
plot([0, params.l1*sin(q1(end)), X_real(end)], [0, -params.l1*cos(q1(end)), Y_real(end)], ...
     'k-o', 'LineWidth', 3, 'MarkerSize', 6, 'MarkerFaceColor','k');
title(['Control en Espacio Operacional (', num2str(N_puntos), ' puntos)']);
xlabel('Posición X [m]'); ylabel('Posición Y [m]');
legend('Trayectoria Deseada', 'Trayectoria Real', 'Brazo Robótico Final');
grid on; axis equal; axis([-0.2 0.8 -0.8 0.2]);

% --- FIGURA 2: Posiciones Angulares (en Grados) ---
figure('Name', 'Posiciones Articulares', 'Color', 'w');
plot(t_sim, q1 * (180/pi), 'LineWidth', 2); hold on;
plot(t_sim, q2 * (180/pi), 'LineWidth', 2);
title('Posiciones Angulares de los Motores');
xlabel('Tiempo [s]'); ylabel('Posición Angular [grados]');
legend('q_1 (Motor 1)', 'q_2 (Motor 2)', 'Location', 'best');
grid on;

% --- FIGURA 3: Velocidades Angulares (en Grados/s) ---
figure('Name', 'Velocidades Articulares', 'Color', 'w');
plot(t_sim, qp1 * (180/pi), 'LineWidth', 2); hold on;
plot(t_sim, qp2 * (180/pi), 'LineWidth', 2);
title('Velocidades Angulares de los Motores');
xlabel('Tiempo [s]'); ylabel('Velocidad Angular [grados/s]');
legend('dq_1/dt (Motor 1)', 'dq_2/dt (Motor 2)', 'Location', 'best');
grid on;


% =========================================================================
% FUNCIÓN 1: move(x, y, tiempo_total, params)
% =========================================================================
function [t_sim, estado] = move(x, y, tiempo_total, params)
    N_puntos = length(x);
    t_span = linspace(0, tiempo_total, N_puntos);
    
    % Condiciones iniciales
    q_inicial = [0; 0.0001; 0; 0]; 
    
    % Calcular velocidades deseadas
    x_dot = gradient(x) ./ gradient(t_span);
    y_dot = gradient(y) ./ gradient(t_span);
    
    opciones = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);
    [t_sim, estado] = ode45(@(t, x_estado) dinamica_cartesiana(t, x_estado, t_span, x, y, x_dot, y_dot, params), t_span, q_inicial, opciones);
end

% =========================================================================
% FUNCIÓN 2: dinamica_cartesiana
% =========================================================================
function xp = dinamica_cartesiana(t, x_estado, t_span, x_traj, y_traj, xdot_traj, ydot_traj, p)
    q1 = x_estado(1);  q2 = x_estado(2);
    qp1 = x_estado(3); qp2 = x_estado(4);
    q = [q1; q2];      qp = [qp1; qp2];
    
    m1 = p.m1; l1 = p.l1; lc1 = p.lc1; I1 = p.I1; b1 = p.b1;
    m2 = p.m2; l2 = p.l2; lc2 = p.lc2; I2 = p.I2; b2 = p.b2; g = p.g;
    
    X_d = interp1(t_span, x_traj, t, 'spline');
    Y_d = interp1(t_span, y_traj, t, 'spline');
    Xd_dot = interp1(t_span, xdot_traj, t, 'spline');
    Yd_dot = interp1(t_span, ydot_traj, t, 'spline');
    
    X_deseada     = [X_d; Y_d];
    X_dot_deseada = [Xd_dot; Yd_dot];
    
    X_real = [l1*sin(q1) + l2*sin(q1+q2);
             -l1*cos(q1) - l2*cos(q1+q2)];
         
    J = [ l1*cos(q1) + l2*cos(q1+q2),  l2*cos(q1+q2);
          l1*sin(q1) + l2*sin(q1+q2),  l2*sin(q1+q2)];
          
    J_dot = [ -l1*sin(q1)*qp1 - l2*sin(q1+q2)*(qp1+qp2), -l2*sin(q1+q2)*(qp1+qp2);
               l1*cos(q1)*qp1 + l2*cos(q1+q2)*(qp1+qp2),  l2*cos(q1+q2)*(qp1+qp2)];
               
    x_dot_real = J * qp; 
    
    theta1 = m1*lc1^2 + m2*l1^2 + m2*lc2^2 + I1 + I2;
    theta2 = l1*m2*lc2;
    theta3 = m2*lc2^2 + I2;
    
    M = [theta1 + 2*theta2*cos(q2), theta3 + theta2*cos(q2);
         theta3 + theta2*cos(q2),   theta3];
         
    C = [-2*theta2*sin(q2)*qp2, -theta2*sin(q2)*qp2;
          theta2*sin(q2)*qp1,    0];
          
    g_vector = [g*(lc1*m1 + m2*l1)*sin(q1) + g*m2*lc2*sin(q1+q2);
                g*m2*lc2*sin(q1+q2)];
                
    fr = [b1*qp1; b2*qp2];
    
    J_inv = inv(J);
    J_invT = J_inv';
    
    Mx  = J_invT * M * J_inv;
    Cx  = J_invT * (C - M * J_inv * J_dot) * J_inv;
    gx  = J_invT * g_vector;
    fex = J_invT * fr;
    
    x_tilde = X_deseada - X_real;
    x_tilde_dot = X_dot_deseada - x_dot_real; 
    
    f_chi = p.Kp * x_tilde + p.Kv * x_tilde_dot + gx;
    
    X_ddot = Mx \ (f_chi - Cx * x_dot_real - gx - fex);
    qpp = J \ (X_ddot - J_dot * qp);
    
    xp = [qp; qpp];
end