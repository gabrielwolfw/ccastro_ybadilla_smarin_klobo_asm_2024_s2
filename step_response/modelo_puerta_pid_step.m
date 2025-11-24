% Parámetros PID
Kp = 45;
Ki = 20;
Kd = 2;

s = tf('s');

% Función de transferencia PID paralelo
Gpid = Kp + Ki/s + Kd*s; % Esto NO usa pid()

% Planta del sistema (modelo de la puerta)
K = 1; a = 2; b = 5;
Gp = K / (s^2 + a*s + b);

% Sistema lazo cerrado
T = feedback(Gpid*Gp, 1);

% Simulación y gráfica como antes
setpoint = 0.5; t = 0:0.01:10;
[y, t_out] = step(setpoint * T, t);

figure;
plot(t_out, y, 'b', 'LineWidth', 2);
hold on;
plot([0 t(end)], [setpoint setpoint], 'r--', 'LineWidth', 1.5);
xlabel('Tiempo [s]'); ylabel('Posición de la puerta [m]');
title('Respuesta al escalón con PID hecho desde cero');
legend('Posición actual','Setpoint');
ylim([0 1.0]); grid on;