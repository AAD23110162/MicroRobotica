% Actividad X: 
% Alejandro Aguirre Díaz-23110162
% MicroRobotica 7-E
% Simulación de Robot Planar 2 GDL - Control por Par Calculado
% Trayectoria: Círculo con Velocidad Tangencial Constante

clear; clc; close all;

% =========================================================================
% 1. PARÁMETROS DEL ROBOT Y CONSTANTES
% =========================================================================
p.m1=23.902; p.l1=0.45; p.lc1=0.091; p.I1=1.266; b1=2.288;
p.m2=3.880;  p.l2=0.45; p.lc2=0.048; p.I2=0.093; b2=0.175; p.g=9.81;

% Matriz de Fricción Viscosa (B)
p.B = [b1, 0; 0, b2]; 

% --- DEFINICIÓN DE TRAYECTO POR VELOCIDAD TANGENCIAL ---
p.Xc = 0.3; p.Yc = -0.3; p.R = 0.15; % Centro y Radio
V_tangencial_deseada = 1.0;          % [m/s] 

p.w = V_tangencial_deseada / p.R;    % Velocidad angular (rad/s)
p.T_periodo = (2*pi) / p.w;          % Tiempo para cerrar el círculo

% Ganancias del Controlador
p.Kp = [250.0,  20.0; 20.0, 100.0];
p.Kv = [35.0,    5.0; 5.0,   15.0];

% =========================================================================
% 2. SIMULACIÓN
% =========================================================================
t_span = [0 p.T_periodo]; 
x0 = [0; 0; 0; 0]; 

disp(['Simulando... Tiempo estimado: ', num2str(p.T_periodo), ' s']);
options = odeset('RelTol',1e-5,'AbsTol',1e-5);
[t_sim, x_est] = ode45(@(t,x) dinamica_robot(t, x, p), t_span, x0, options);

% =========================================================================
% 3. CÁLCULO DE RESULTADOS
% =========================================================================
N = length(t_sim);
q_real = x_est(:, 1:2); qp_real = x_est(:, 3:4);
q_des = zeros(N, 2);   qp_des = zeros(N, 2);
X_real = zeros(N, 1);  Y_real = zeros(N, 1);
X_des = zeros(N, 1);   Y_des = zeros(N, 1);

for k = 1:N
    t = t_sim(k);
    Xd = p.Xc + p.R*cos(p.w*t); Yd = p.Yc + p.R*sin(p.w*t);
    Vd = [-p.R*p.w*sin(p.w*t); p.R*p.w*cos(p.w*t)];
    X_des(k) = Xd; Y_des(k) = Yd;
    
    D = (Xd^2 + Yd^2 - p.l1^2 - p.l2^2) / (2 * p.l1 * p.l2);
    D = max(min(D, 1), -1); q2d = acos(D);
    q1d = atan2((p.l1+p.l2*cos(q2d))*Xd + (p.l2*sin(q2d))*Yd, ...
                (p.l2*sin(q2d))*Xd - (p.l1+p.l2*cos(q2d))*Yd);
    q_des(k,:) = [q1d, q2d];
    
    J = [p.l1*cos(q1d)+p.l2*cos(q1d+q2d), p.l2*cos(q1d+q2d);
         p.l1*sin(q1d)+p.l2*sin(q1d+q2d), p.l2*sin(q1d+q2d)];
    qp_des(k,:) = (J \ Vd)';
    
    X_real(k) = p.l1*sin(q_real(k,1)) + p.l2*sin(q_real(k,1)+q_real(k,2));
    Y_real(k) = -p.l1*cos(q_real(k,1)) - p.l2*cos(q_real(k,1)+q_real(k,2));
end

e_vel = qp_des - qp_real;
rad2deg = 180/pi;

% =========================================================================
% 4. GRÁFICAS (2 FIGURAS)
% =========================================================================

% --- FIGURA 1: TRAYECTORIA Y ERROR DE VELOCIDAD ---
figure('Name', 'Analisis de Trayectoria y Error de Velocidad', 'Color', 'w');

subplot(2,1,1);
plot(X_des, Y_des, 'k--', 'LineWidth', 2); hold on;
plot(X_real, Y_real, 'b-', 'LineWidth', 1.5);
plot(X_real(1), Y_real(1), 'ro', 'MarkerFaceColor', 'r'); 
plot(X_real(end), Y_real(end), 'gs', 'MarkerFaceColor', 'g');
title(['Trayectoria Cartesiana (V = ', num2str(V_tangencial_deseada), ' m/s)'], 'Interpreter', 'none');
xlabel('X [m]'); ylabel('Y [m]'); legend('Deseada', 'Real', 'Inicio', 'Fin');
grid on; axis equal;

subplot(2,1,2);
plot(t_sim, e_vel(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, e_vel(:,2), 'r', 'LineWidth', 1.5);
title('Convergencia del Error de Velocidad ($\dot{\tilde{q}} \rightarrow 0$)', 'Interpreter', 'latex');
xlabel('Tiempo [s]'); ylabel('Error [rad/s]'); 
legend({'$\dot{q}_1$', '$\dot{q}_2$'}, 'Interpreter', 'latex');
grid on;

% --- FIGURA 2: SEGUIMIENTO ARTICULAR ---
figure('Name', 'Seguimiento de Articulaciones', 'Color', 'w');

% Posición q1
subplot(2,2,1);
plot(t_sim, q_des(:,1)*rad2deg, 'k--', t_sim, q_real(:,1)*rad2deg, 'b', 'LineWidth', 1.2);
title('Posicion $q_1$', 'Interpreter', 'latex');
ylabel('Grados [^o]'); grid on;

% Posición q2
subplot(2,2,3);
plot(t_sim, q_des(:,2)*rad2deg, 'k--', t_sim, q_real(:,2)*rad2deg, 'r', 'LineWidth', 1.2);
title('Posicion $q_2$', 'Interpreter', 'latex');
xlabel('Tiempo [s]'); ylabel('Grados [^o]'); grid on;

% Velocidad qp1
subplot(2,2,2);
plot(t_sim, qp_des(:,1), 'k--', t_sim, qp_real(:,1), 'b', 'LineWidth', 1.2);
title('Velocidad $\dot{q}_1$', 'Interpreter', 'latex');
ylabel('rad/s'); grid on;

% Velocidad qp2
subplot(2,2,4);
plot(t_sim, qp_des(:,2), 'k--', t_sim, qp_real(:,2), 'r', 'LineWidth', 1.2);
title('Velocidad $\dot{q}_2$', 'Interpreter', 'latex');
xlabel('Tiempo [s]'); ylabel('rad/s'); grid on;

% =========================================================================
% FUNCIÓN DE DINÁMICA
% =========================================================================
function dx = dinamica_robot(t, x, p)
    q = x(1:2); qp = x(3:4);
    
    t1 = p.m1*p.lc1^2 + p.m2*p.l1^2 + p.m2*p.lc2^2 + p.I1 + p.I2;
    t2 = p.l1*p.m2*p.lc2; t3 = p.m2*p.lc2^2 + p.I2;
    t4 = p.g*(p.lc1*p.m1 + p.m2*p.l1); t5 = p.g*p.m2*p.lc2;

    M = [t1 + 2*t2*cos(q(2)), t3 + t2*cos(q(2)); t3 + t2*cos(q(2)), t3];
    C = [-2*t2*sin(q(2))*qp(2), -t2*sin(q(2))*qp(2); t2*sin(q(2))*qp(1), 0];
    G = [t4*sin(q(1)) + t5*sin(q(1)+q(2)); t5*sin(q(1)+q(2))];

    Xd = p.Xc + p.R*cos(p.w*t); Yd = p.Yc + p.R*sin(p.w*t);
    Vd = [-p.R*p.w*sin(p.w*t); p.R*p.w*cos(p.w*t)];
    Ad = [-p.R*p.w^2*cos(p.w*t); -p.R*p.w^2*sin(p.w*t)];

    D = (Xd^2 + Yd^2 - p.l1^2 - p.l2^2) / (2 * p.l1 * p.l2);
    D = max(min(D, 1), -1); q2d = acos(D);
    q1d = atan2((p.l1+p.l2*cos(q2d))*Xd + (p.l2*sin(q2d))*Yd, ...
                (p.l2*sin(q2d))*Xd - (p.l1+p.l2*cos(q2d)) * Yd);
    qd = [q1d; q2d];
    J = [p.l1*cos(q1d)+p.l2*cos(q1d+q2d), p.l2*cos(q1d+q2d);
         p.l1*sin(q1d)+p.l2*sin(q1d+q2d), p.l2*sin(q1d+q2d)];
    qpd = J \ Vd;
    s1 = sin(q1d); s12 = sin(q1d+q2d); c1 = cos(q1d); c12 = cos(q1d+q2d);
    Jp = [-(p.l1*s1+p.l2*s12)*qpd(1)-p.l2*s12*qpd(2), -p.l2*s12*(qpd(1)+qpd(2));
           (p.l1*c1+p.l2*c12)*qpd(1)+p.l2*c12*qpd(2),  p.l2*c12*(qpd(1)+qpd(2))];
    qppd = J \ (Ad - Jp*qpd);

    e = qd - q; ep = qpd - qp;
    u = qppd + p.Kv * ep + p.Kp * e; 
    tau = M * u + C * qp + G + p.B * qp;
    qpp = M \ (tau - C*qp - G - p.B*qp);
    dx = [qp; qpp];
end