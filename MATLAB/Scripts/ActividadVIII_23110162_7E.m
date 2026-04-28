% Actividad VIII: Índice de desempeño
% Alejandro Aguirre Díaz-23110162
% MicroRobotica 7-E
% Comparativas de diferentes leyes de control para trazar un circulo con un
% brazo roboico de 2GDL
clear; clc; close all;

% --- CONFIGURACIÓN DE LA SIMULACIÓN ---
t_span = [0 5];          % Tiempo de simulación (5 segundos)
x0 = [0; 0; 0; 0];  % Condiciones iniciales

Controladores = {'PD', 'TANH', 'ATAN'};
Resultados = struct();

% Parámetros del círculo (Deben coincidir con dinamica_comparativa.m)
Xc = 0.3; Yc = -0.4; R = 0.15; T_periodo = 5.0; w = (2*pi) / T_periodo;
l1 = 0.45; l2 = 0.45;

disp('Simulando y calculando Índice de Desempeño L2...');

for i = 1:length(Controladores)
    tipo = Controladores{i};
    fprintf('  Procesando %s...\n', tipo);
    
    % 1. Resolver ODE
    f_ode = @(t,x) dinamica_comparativa(t, x, tipo);
    [t, x] = ode45(f_ode, t_span, x0);
    
    % 2. Extraer Estados
    q_real = x(:, 1:2);
    qp_real = x(:, 3:4);
    
    % 3. Cinemática Directa (Posición Cartesiana Real)
    X_real = l1 .* sin(q_real(:,1)) + l2 .* sin(q_real(:,1) + q_real(:,2));
    Y_real = -l1 .* cos(q_real(:,1)) - l2 .* cos(q_real(:,1) + q_real(:,2));
    
    % 4. Recalcular la Referencia Articular (qd) para todo t
    Xd = Xc + R .* cos(w .* t);
    Yd = Yc + R .* sin(w .* t);
    
    D = (Xd.^2 + Yd.^2 - l1^2 - l2^2) / (2 * l1 * l2);
    D = max(min(D, 1), -1); 
    q2_d = acos(D); 
    A = l1 + l2 .* cos(q2_d);
    B = l2 .* sin(q2_d);
    q1_d = atan2(A.*Xd + B.*Yd, B.*Xd - A.*Yd);
    
    % 5. Cálculo del Error Articular (q_tilde)
    q_tilde_1 = q1_d - q_real(:,1);
    q_tilde_2 = q2_d - q_real(:,2);
    
    % Norma al cuadrado en cada instante ||q_tilde(t)||^2
    norm_q_tilde_sq = q_tilde_1.^2 + q_tilde_2.^2;
    norma_instantanea = sqrt(norm_q_tilde_sq);
    
    % 6. CÁLCULO DEL ÍNDICE L2 (Según la fórmula)
    T_sim = t(end); 
    integral_error = trapz(t, norm_q_tilde_sq); 
    indice_L2 = sqrt((1 / T_sim) * integral_error);
    
    % Guardar en estructura
    Resultados.(tipo).t = t;
    Resultados.(tipo).q1 = q_real(:,1) * (180/pi);
    Resultados.(tipo).q2 = q_real(:,2) * (180/pi);
    Resultados.(tipo).qp1 = qp_real(:,1);
    Resultados.(tipo).qp2 = qp_real(:,2);
    Resultados.(tipo).X = X_real;
    Resultados.(tipo).Y = Y_real;
    Resultados.(tipo).norma_inst = norma_instantanea;
    Resultados.(tipo).indice_L2 = indice_L2;
end

% --- MOSTRAR RESULTADOS NUMÉRICOS EN CONSOLA ---
fprintf('\n=== ÍNDICE DE DESEMPEÑO L2 (Menor es mejor) ===\n');
L2_valores = zeros(1, 3);
for i = 1:length(Controladores)
    tipo = Controladores{i};
    L2_valores(i) = Resultados.(tipo).indice_L2;
    fprintf('Controlador %s: %.6f rad\n', tipo, L2_valores(i));
end

% --- GENERAR REFERENCIA IDEAL CARTESIANA ---
t_ideal = linspace(0, T_sim, 500);
X_ideal = Xc + R .* cos(w .* t_ideal);
Y_ideal = Yc + R .* sin(w .* t_ideal);

disp('Generando todas las gráficas...');

% =========================================================================
% FIGURA 1: TRAYECTORIA CARTESIANA
% =========================================================================
figure('Name', 'Trayectoria Cartesiana', 'Color', 'w');
hold on; grid on;
plot(X_ideal, Y_ideal, 'k--', 'LineWidth', 2, 'DisplayName', 'Ref. Ideal');
plot(Resultados.PD.X, Resultados.PD.Y, 'b', 'LineWidth', 1.5, 'DisplayName', 'PD');
plot(Resultados.TANH.X, Resultados.TANH.Y, 'r', 'LineWidth', 1.5, 'DisplayName', 'TANH');
plot(Resultados.ATAN.X, Resultados.ATAN.Y, 'g', 'LineWidth', 1.5, 'DisplayName', 'ATAN');
title('Seguimiento de Trayectoria Circular (X-Y)');
xlabel('X [m]'); ylabel('Y [m]');
legend('Location', 'best'); axis equal;

% =========================================================================
% FIGURA 2: POSICIÓN ARTICULAR
% =========================================================================
figure('Name', 'Posición Articular', 'Color', 'w');
subplot(2,1,1); hold on; grid on;
plot(Resultados.PD.t, Resultados.PD.q1, 'b', 'LineWidth', 1.5);
plot(Resultados.TANH.t, Resultados.TANH.q1, 'r', 'LineWidth', 1.5);
plot(Resultados.ATAN.t, Resultados.ATAN.q1, 'g', 'LineWidth', 1.5);
title('Posición Eslabón 1 (q1)', 'Interpreter', 'latex', 'FontSize', 12); 
ylabel('[Grados]');
legend('PD', 'TANH', 'ATAN');

subplot(2,1,2); hold on; grid on;
plot(Resultados.PD.t, Resultados.PD.q2, 'b', 'LineWidth', 1.5);
plot(Resultados.TANH.t, Resultados.TANH.q2, 'r', 'LineWidth', 1.5);
plot(Resultados.ATAN.t, Resultados.ATAN.q2, 'g', 'LineWidth', 1.5);
title('Posición Eslabón 2 (q2)', 'Interpreter', 'latex', 'FontSize', 12); 
xlabel('Tiempo [s]'); ylabel('[Grados]');

% =========================================================================
% FIGURA 3: VELOCIDAD ARTICULAR (Corregido)
% =========================================================================
figure('Name', 'Velocidad Articular', 'Color', 'w');
subplot(2,1,1); hold on; grid on;
plot(Resultados.PD.t, Resultados.PD.qp1, 'b', 'LineWidth', 1.5);
plot(Resultados.TANH.t, Resultados.TANH.qp1, 'r', 'LineWidth', 1.5);
plot(Resultados.ATAN.t, Resultados.ATAN.qp1, 'g', 'LineWidth', 1.5);
title('Velocidad Eslabón 1 ', 'Interpreter', 'latex', 'FontSize', 12); 
ylabel('[rad/s]');
legend('PD', 'TANH', 'ATAN');

subplot(2,1,2); hold on; grid on;
plot(Resultados.PD.t, Resultados.PD.qp2, 'b', 'LineWidth', 1.5);
plot(Resultados.TANH.t, Resultados.TANH.qp2, 'r', 'LineWidth', 1.5);
plot(Resultados.ATAN.t, Resultados.ATAN.qp2, 'g', 'LineWidth', 1.5);
title('Velocidad Eslabón 2 ', 'Interpreter', 'latex', 'FontSize', 12); 
xlabel('Tiempo [s]'); ylabel('[rad/s]');

% =========================================================================
% FIGURA 4: EVOLUCIÓN DEL ERROR ARTICULAR (Corregido)
% =========================================================================
figure('Name', 'Evolución del Error', 'Color', 'w');
hold on; grid on;
plot(Resultados.PD.t, Resultados.PD.norma_inst, 'b', 'LineWidth', 1.5, 'DisplayName', 'PD');
plot(Resultados.TANH.t, Resultados.TANH.norma_inst, 'r', 'LineWidth', 1.5, 'DisplayName', 'TANH');
plot(Resultados.ATAN.t, Resultados.ATAN.norma_inst, 'g', 'LineWidth', 1.5, 'DisplayName', 'ATAN');
title('Evolución de la Norma del Error Articular ', 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Tiempo [s]'); ylabel('Error [Radianes]');
legend('Location', 'best');

% =========================================================================
% FIGURA 5: COMPARATIVA DEL ÍNDICE DE DESEMPEÑO L2 (Corregido)
% =========================================================================
figure('Name', 'Índice de Desempeño L2', 'Color', 'w');
bar_h = bar(L2_valores, 'FaceColor', 'flat');
bar_h.CData(1,:) = [0 0 1]; % Azul
bar_h.CData(2,:) = [1 0 0]; % Rojo
bar_h.CData(3,:) = [0 1 0]; % Verde

set(gca, 'xticklabel', Controladores);
title('Índice de Desempeño 2 (Fórmula RMS)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Valor $\mathcal{L}_2$ [Radianes]', 'Interpreter', 'latex', 'FontSize', 12);
grid on;

for i = 1:length(L2_valores)
    text(i, L2_valores(i), num2str(L2_valores(i), '%.4f'), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontWeight', 'bold');
end


function xp = dinamica_comparativa(t, x, tipo_control)
    % Estados
    q = [x(1); x(2)];
    qp = [x(3); x(4)];
    
    % --- Parámetros del Robot ---
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
    
    % --- GENERACIÓN DE TRAYECTORIA (CÍRCULO) ---
    Xc = 0.3; Yc = -0.4; R = 0.15; T_periodo = 5.0;
    w = (2*pi) / T_periodo;
    
    Xd = Xc + R * cos(w * t);
    Yd = Yc + R * sin(w * t);
    
    % --- CINEMÁTICA INVERSA ---
    D = (Xd^2 + Yd^2 - l1^2 - l2^2) / (2 * l1 * l2);
    D = max(min(D, 1), -1); 
    q2_d = acos(D); 
    A = l1 + l2 * cos(q2_d);
    B = l2 * sin(q2_d);
    q1_d = atan2(A*Xd + B*Yd, B*Xd - A*Yd);
    
    qd = [q1_d; q2_d];
    q_tilde = qd - q; % Error de posición
    
% --- SELECCIÓN DE LEY DE CONTROL ---
    % Ganancias de Velocidad con acoplamiento cruzado (Simétrica y Definida Positiva)
    Kv = [35.0,  5.0; 
           5.0, 15.0]; 
    
    switch tipo_control
        case 'PD'
            % Ganancias PD con acoplamiento cruzado
            Kp = [250.0,  20.0; 
                   20.0, 100.0];
            tau = Kp * q_tilde - Kv * qp + par_grav;
            
        case 'TANH'
            % Kp es el límite de torque máximo, agregamos acoplamiento cruzado suave
            Kp = [150.0,  15.0; 
                   15.0,  80.0]; 
            tau = Kp * tanh(q_tilde) - Kv * qp + par_grav;
            
        case 'ATAN'
            % Ajuste similar para ATAN
            Kp = [150.0,  15.0; 
                   15.0,  80.0];
            tau = Kp * atan(q_tilde) - Kv * qp + par_grav;
    end
    
    % --- Dinámica ---
    qpp = M \ (tau - C*qp - par_grav - fr);
    xp = [qp; qpp]; % Devuelve [velocidad; aceleración]
end