clc, clear all, close all
A=[0 1; 0 -0.625];
B=[0; 0.0327];
C=[1 0];

A_hat = [A, zeros(2,1); -C, 0];
B_hat = [B; 0];
B_ref = [0; 0; 1];

% Ganancias LQR
Q1 = diag([500, 10, 1000]); R1 = 1;
K1 = lqr(A_hat, B_hat, Q1, R1);
Q2 = diag([500, 10, 5000]); R2 = 1;
K2 = lqr(A_hat, B_hat, Q2, R2);
Q3 = diag([500, 800, 1500]); R3 = 2;
K3 = lqr(A_hat, B_hat, Q3, R3);

dt = 0.01; 
t = 0:dt:15; 
SP = 10;
% Inicializar estados (ahora de 3 dimensiones: [pos; vel; error_int])
x1 = [0;0;0]; x2 = [0;0;0]; x3 = [0;0;0];

for i = 1:length(t)
    % Caso 1
    u1(i) = -K1 * x1;
    x1 = x1 + (A_hat*x1 + B_hat*u1(i) + B_ref*SP)*dt;
    h1(i) = x1(1);
    % Caso 2
    u2(i) = -K2 * x2;
    x2 = x2 + (A_hat*x2 + B_hat*u2(i) + B_ref*SP)*dt;
    h2(i) = x2(1);
    % Caso 3
    u3(i) = -K3 * x3;
    x3 = x3 + (A_hat*x3 + B_hat*u3(i) + B_ref*SP)*dt;
    h3(i) = x3(1);
end
figure;
subplot(2,1,1);
plot(t, h1, 'b', t, h2, 'r', t, h3, 'g', 'LineWidth', 1.5);
yline(SP, '--k'); title('Efecto de variar Q y R');
legend('Balanceado', 'Agresivo (Q alto)', 'Conservador (R alto)');
grid on; ylabel('Angulo');

subplot(2,1,2);
plot(t, u1, 'b', t, u2, 'r', t, u3, 'g', 'LineWidth', 1.5);
title('Esfuerzo de Control v_i(t)');
legend('Balanceado', 'Agresivo', 'Conservador');
grid on; ylabel('PWM');