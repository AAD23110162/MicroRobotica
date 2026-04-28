% Actividad VII: Trayectoria de un círculo
% Alejandro Aguirre Díaz-23110162
% MicroRobotica 7-E
% Robot Planar 2GDL - Control PD + Compensación de Gravedad

clear; clc; close all;

% --- CONFIGURACIÓN DE LA SIMULACIÓN ---
t_span = [0 5];      
x0 = [0; 0; 0; 0];  
l1 = 0.45; l2 = 0.45;

% 1. Simulación
fprintf('Simulando PD + Compensación de Gravedad...\n');
[t, x] = ode45(@(t,x) dinamica_pd_gravedad(t, x), t_span, x0);

% 2. Extracción de datos
q_real = x(:, 1:2);
qp_real = x(:, 3:4);

% 3. Cinemática Directa para Trayectoria Cartesiana
X_real = l1 .* sin(q_real(:,1)) + l2 .* sin(q_real(:,1) + q_real(:,2));
Y_real = -l1 .* cos(q_real(:,1)) - l2 .* cos(q_real(:,1) + q_real(:,2));

% 4. Referencia Cartesiana Ideal
Xc = 0.3; Yc = -0.4; R = 0.15; T_periodo = 5.0; w = (2*pi) / T_periodo;
Xd = Xc + R .* cos(w .* t);
Yd = Yc + R .* sin(w .* t);

% --- GRÁFICA 1: TRAYECTORIA CARTESIANA (X-Y) ---
figure('Name', 'Trayectoria Cartesiana', 'Color', 'w');
plot(Xd, Yd, 'k--', 'LineWidth', 2, 'DisplayName', 'Referencia');
hold on; grid on;
plot(X_real, Y_real, 'b', 'LineWidth', 1.5, 'DisplayName', 'PD + Grav');
title('Seguimiento de Trayectoria Circular');
xlabel('X [m]'); ylabel('Y [m]');
legend; axis equal;

% --- GRÁFICA 2: POSICIONES ARTICULARES (q1, q2) ---
figure('Name', 'Posiciones Articulares', 'Color', 'w');
subplot(2,1,1);
plot(t, q_real(:,1) * (180/pi), 'r', 'LineWidth', 1.5);
grid on; title('Posición Articular q1', 'Interpreter', 'latex');
ylabel('Grados [$^\circ$]', 'Interpreter', 'latex');

subplot(2,1,2);
plot(t, q_real(:,2) * (180/pi), 'b', 'LineWidth', 1.5);
grid on; title('Posición Articular q2', 'Interpreter', 'latex');
xlabel('Tiempo [s]'); ylabel('Grados [$^\circ$]', 'Interpreter', 'latex');

% --- GRÁFICA 3: VELOCIDADES ARTICULARES (qp1, qp2) ---
figure('Name', 'Velocidades Articulares', 'Color', 'w');
subplot(2,1,1);
plot(t, qp_real(:,1), 'r', 'LineWidth', 1.5);
grid on; title('Velocidad Articular $\dot{q}_1$', 'Interpreter', 'latex');
ylabel('Rad/s');

subplot(2,1,2);
plot(t, qp_real(:,2), 'b', 'LineWidth', 1.5);
grid on; title('Velocidad Articular $\dot{q}_2$', 'Interpreter', 'latex');
xlabel('Tiempo [s]'); ylabel('Rad/s');

% =========================================================================
% FUNCIÓN DE DINÁMICA CON LEY DE CONTROL PD + G
% =========================================================================
function xp = dinamica_pd_gravedad(t, x)
    q = [x(1); x(2)];
    qp = [x(3); x(4)];
    
    % Parámetros del Robot
    m1=23.902; l1=0.45; lc1=0.091; I1=1.266; b1=2.288;
    m2=3.880; l2=0.45; lc2=0.048; I2=0.093; b2=0.175; g=9.81;
    
    theta1 = m1*lc1^2 + m2*l1^2 + m2*lc2^2 + I1 + I2;
    theta2 = l1*m2*lc2;
    theta3 = m2*lc2^2 + I2;
    theta4 = g*(lc1*m1 + m2*l1);
    theta5 = g*m2*lc2;
    
    M = [theta1 + 2*theta2*cos(q(2)), theta3 + theta2*cos(q(2));
         theta3 + theta2*cos(q(2)),   theta3];
    C = [-2*theta2*sin(q(2))*qp(2), -theta2*sin(q(2))*qp(2);
          theta2*sin(q(2))*qp(1),    0];
    par_grav = [theta4*sin(q(1)) + theta5*sin(q(1)+q(2));
                theta5*sin(q(1)+q(2))];
    fr = [b1*qp(1); b2*qp(2)];
    
    % Referencia Circular
    Xc = 0.3; Yc = -0.4; R = 0.15; T_periodo = 5.0; w = (2*pi) / T_periodo;
    Xd_t = Xc + R * cos(w * t);
    Yd_t = Yc + R * sin(w * t);
    
    % Cinemática Inversa
    D = (Xd_t^2 + Yd_t^2 - l1^2 - l2^2) / (2 * l1 * l2);
    D = max(min(D, 1), -1); 
    q2_d = acos(D); 
    A = l1 + l2 * cos(q2_d);
    B = l2 * sin(q2_d);
    q1_d = atan2(A*Xd_t + B*Yd_t, B*Xd_t - A*Yd_t);
    
    qd = [q1_d; q2_d];
    q_tilde = qd - q;
    
    % LEY DE CONTROL: PD + Compensación de Gravedad
    Kp = [450, 0; 0, 250]; % Ganancias Proporcionales
    Kv = [60, 0; 0, 40];   % Ganancias Derivativas
    
    % Torque de control
    tau = Kp * q_tilde - Kv * qp + par_grav;
    
    % Dinámica
    qpp = M \ (tau - C*qp - par_grav - fr);
    xp = [qp; qpp];
end