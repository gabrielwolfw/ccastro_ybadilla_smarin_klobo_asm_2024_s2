%% Sistema de Control PID para Puerta Corrediza
% CE 1110 - Análisis de Señales Mixtas
% Motor DC con Controlador PID
% Tres velocidades de operación

clear all; close all; clc;

fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║  CONTROL DE VELOCIDAD - PUERTA CORREDIZA CON PID      ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

%% 1. MODELO DEL MOTOR DC
% Modelo típico de motor DC de segundo orden:
% G(s) = K / (s² + as + b)
% Donde: K = ganancia, a = coef. amortiguamiento, b = freq. natural²

s = tf('s');

% Parámetros del motor DC (basados en motor típico de 12V)
K = 1;      % Ganancia normalizada
a = 2;      % Coeficiente de amortiguamiento  
b = 5;      % Frecuencia natural al cuadrado

% Función de transferencia de la planta (Motor DC)
Gp = K / (s^2 + a*s + b);

fprintf('═══ PLANTA: MOTOR DC ═══\n');
fprintf('Función de transferencia:\n');
disp(Gp);

% Características de la planta
fprintf('Parámetros del motor:\n');
fprintf('  K (ganancia) = %.2f\n', K);
fprintf('  a (amortiguamiento) = %.2f\n', a);
fprintf('  b (freq. natural²) = %.2f\n\n', b);

%% 2. ANÁLISIS DE POLOS Y CEROS - PLANTA
fprintf('═══ ANÁLISIS: PLANTA SIN CONTROLADOR ═══\n');

[z_planta, p_planta, k_planta] = zpkdata(Gp, 'v');

fprintf('Polos de la planta:\n');
for i = 1:length(p_planta)
    if imag(p_planta(i)) ~= 0
        fprintf('  p%d = %.4f %+.4fi\n', i, real(p_planta(i)), imag(p_planta(i)));
    else
        fprintf('  p%d = %.4f\n', i, p_planta(i));
    end
end

fprintf('\nCeros de la planta:\n');
if isempty(z_planta)
    fprintf('  (No tiene ceros)\n');
else
    for i = 1:length(z_planta)
        fprintf('  z%d = %.4f\n', i, z_planta(i));
    end
end
fprintf('\n');

%% 3. CONTROLADOR PID
% Parámetros ajustados para un buen desempeño
Kp = 30;   % Ganancia proporcional - respuesta rápida
Ki = 20;   % Ganancia integral - elimina error estado estacionario
Kd = 3;    % Ganancia derivativa - reduce sobrepaso

% Controlador PID: C(s) = Kp + Ki/s + Kd*s
Gpid = Kp + Ki/s + Kd*s;

fprintf('═══ CONTROLADOR PID ═══\n');
fprintf('Parámetros:\n');
fprintf('  Kp (Proporcional) = %.2f\n', Kp);
fprintf('  Ki (Integral) = %.2f\n', Ki);
fprintf('  Kd (Derivativo) = %.2f\n\n', Kd);

disp(Gpid);

%% 4. SISTEMA EN LAZO CERRADO
% Retroalimentación unitaria H(s) = 1
H = 1;

% T(s) = (C(s)*G(s)) / (1 + C(s)*G(s)*H(s))
T_closed = (Gpid * Gp) / (1 + Gpid * Gp * H);

fprintf('═══ SISTEMA EN LAZO CERRADO ═══\n');
fprintf('Función de transferencia T(s):\n');
disp(T_closed);

%% 5. ANÁLISIS DE POLOS Y CEROS - SISTEMA CON PID
fprintf('═══ ANÁLISIS: SISTEMA CON PID ═══\n');

[z_closed, p_closed, k_closed] = zpkdata(T_closed, 'v');

fprintf('Polos del sistema con PID:\n');
for i = 1:length(p_closed)
    if imag(p_closed(i)) ~= 0
        fprintf('  p%d = %.4f %+.4fi  |  Magnitud: %.4f\n', ...
            i, real(p_closed(i)), imag(p_closed(i)), abs(p_closed(i)));
    else
        fprintf('  p%d = %.4f\n', i, p_closed(i));
    end
end

fprintf('\nCeros del sistema con PID:\n');
if isempty(z_closed)
    fprintf('  (No tiene ceros)\n');
else
    for i = 1:length(z_closed)
        if imag(z_closed(i)) ~= 0
            fprintf('  z%d = %.4f %+.4fi\n', i, real(z_closed(i)), imag(z_closed(i)));
        else
            fprintf('  z%d = %.4f\n', i, z_closed(i));
        end
    end
end

% Verificar estabilidad
fprintf('\nEstabilidad del sistema:\n');
if all(real(p_closed) < 0)
    fprintf('  ✓ SISTEMA ESTABLE (todos los polos con parte real negativa)\n\n');
else
    fprintf('  ✗ SISTEMA INESTABLE\n\n');
end

%% 6. DIAGRAMAS DE POLOS Y CEROS
figure('Position', [100 50 1400 500]);

% Planta sin controlador
subplot(1,3,1);
pzmap(Gp);
title('Polos y Ceros: PLANTA (Motor DC)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
legend('Polos', 'Location', 'best');

% Controlador PID
subplot(1,3,2);
pzmap(Gpid);
title('Polos y Ceros: CONTROLADOR PID', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
legend('Polos', 'Ceros', 'Location', 'best');

% Sistema completo en lazo cerrado
subplot(1,3,3);
pzmap(T_closed);
title('Polos y Ceros: SISTEMA COMPLETO', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
legend('Polos', 'Ceros', 'Location', 'best');

sgtitle('Análisis de Polos y Ceros del Sistema de Control', 'FontSize', 14, 'FontWeight', 'bold');

%% 7. DEFINICIÓN DE LAS 3 VELOCIDADES
% Setpoints para las tres velocidades de operación
velocidad_baja = 0.5;    % 50 RPM o 0.5 metros
velocidad_media = 0.75;  % 75 RPM o 0.75 metros
velocidad_alta = 1.0;    % 100 RPM o 1.0 metro

setpoints = [velocidad_baja, velocidad_media, velocidad_alta];
colores = {'b', 'g', 'r'};
nombres = {'Velocidad BAJA (50%)', 'Velocidad MEDIA (75%)', 'Velocidad ALTA (100%)'};

fprintf('═══ VELOCIDADES DE OPERACIÓN ═══\n');
fprintf('Setpoint Bajo:  %.2f\n', velocidad_baja);
fprintf('Setpoint Medio: %.2f\n', velocidad_media);
fprintf('Setpoint Alto:  %.2f\n\n', velocidad_alta);

%% 8. SIMULACIÓN CON RESPUESTA AL IMPULSO
% Usando impulse() dividiendo entre 's' para obtener respuesta al escalón
t = 0:0.01:10;

figure('Position', [100 50 1400 900]);

for i = 1:3
    setpoint = setpoints(i);
    
    % Respuesta usando impulse (escalón = impulso / s)
    [y, t_out] = impulse(setpoint * T_closed / s, t);
    
    % Subplot para cada velocidad
    subplot(2, 3, i);
    plot(t_out, y, colores{i}, 'LineWidth', 2.5);
    hold on;
    plot([0 t_out(end)], [setpoint setpoint], 'k--', 'LineWidth', 1.5);
    yline(setpoint*1.02, 'r:', 'LineWidth', 1); % +2%
    yline(setpoint*0.98, 'r:', 'LineWidth', 1); % -2%
    xlabel('Tiempo [s]', 'FontSize', 11);
    ylabel('Posición [m] o Velocidad [RPM]', 'FontSize', 11);
    title(nombres{i}, 'FontSize', 12, 'FontWeight', 'bold');
    legend('Respuesta', 'Setpoint', 'Banda ±2%', 'Location', 'best');
    grid on;
    ylim([0, max(setpoint*1.15, 1.1)]);
    
    % Calcular características
    info = stepinfo(setpoint * T_closed);
    
    % Tabla de características
    subplot(2, 3, i+3);
    axis off;
    
    texto = {
        '╔════════════════════════════════╗'
        '║  CARACTERÍSTICAS DEL SISTEMA   ║'
        '╚════════════════════════════════╝'
        ' '
        sprintf('Setpoint: %.3f', setpoint)
        sprintf('Valor final: %.4f', y(end))
        sprintf('Error estado estac.: %.2f%%', abs(y(end)-setpoint)/setpoint*100)
        ' '
        sprintf('Tiempo de subida: %.3f s', info.RiseTime)
        sprintf('Tiempo de pico: %.3f s', info.PeakTime)
        sprintf('Tiempo establec. (2%%): %.3f s', info.SettlingTime)
        ' '
        sprintf('Sobrepaso: %.2f%%', info.Overshoot)
        sprintf('Valor pico: %.4f', info.Peak)
    };
    
    text(0.05, 0.5, texto, 'FontSize', 10, 'FontName', 'Courier', ...
        'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
end

sgtitle('Respuesta del Sistema con PID para Tres Velocidades', 'FontSize', 14, 'FontWeight', 'bold');

%% 9. COMPARACIÓN DE LAS 3 VELOCIDADES EN UNA GRÁFICA
figure('Position', [100 50 1200 500]);

subplot(1,2,1);
hold on;
for i = 1:3
    setpoint = setpoints(i);
    [y, t_out] = impulse(setpoint * T_closed / s, t);
    plot(t_out, y, colores{i}, 'LineWidth', 2.5, 'DisplayName', nombres{i});
    plot([0 t_out(end)], [setpoint setpoint], '--', 'Color', colores{i}, ...
        'LineWidth', 1.2, 'HandleVisibility', 'off');
end
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Posición [m] o Velocidad [RPM]', 'FontSize', 11);
title('Comparación de Respuestas - Tres Velocidades', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;

% Zoom en la región inicial
subplot(1,2,2);
hold on;
for i = 1:3
    setpoint = setpoints(i);
    [y, t_out] = impulse(setpoint * T_closed / s, t);
    plot(t_out, y, colores{i}, 'LineWidth', 2.5, 'DisplayName', nombres{i});
end
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Posición [m] o Velocidad [RPM]', 'FontSize', 11);
title('Zoom: Respuesta Transitoria', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
xlim([0 3]);

%% 10. ANÁLISIS DE DESEMPEÑO
fprintf('═══ ANÁLISIS DE DESEMPEÑO ═══\n\n');

for i = 1:3
    setpoint = setpoints(i);
    info = stepinfo(setpoint * T_closed);
    
    fprintf('%s\n', nombres{i});
    fprintf('  Setpoint: %.3f\n', setpoint);
    fprintf('  Tiempo de subida: %.4f s\n', info.RiseTime);
    fprintf('  Tiempo al pico: %.4f s\n', info.PeakTime);
    fprintf('  Tiempo de establecimiento (2%%): %.4f s\n', info.SettlingTime);
    fprintf('  Sobrepaso: %.3f %%\n', info.Overshoot);
    fprintf('  Valor pico: %.4f\n', info.Peak);
    fprintf('  Valor final: %.4f\n', y(end));
    fprintf('\n');
end

%% 11. FRECUENCIA DE MUESTREO
fprintf('═══ DISEÑO DIGITAL DEL CONTROLADOR ═══\n');

bw = bandwidth(T_closed);
freq_hz = bw / (2*pi);

fprintf('Ancho de banda: %.3f rad/s (%.3f Hz)\n', bw, freq_hz);
fprintf('\nFrecuencia de muestreo:\n');
fprintf('  Mínima (10×BW): %.2f Hz\n', 10*freq_hz);
fprintf('  Recomendada (20×BW): %.2f Hz\n', 20*freq_hz);

Ts = 1/(20*freq_hz);
fprintf('  Ts recomendado: %.5f s\n\n', Ts);

%% 12. DISCRETIZACIÓN
% Sistema discreto usando Zero-Order Hold
T_closed_d = c2d(T_closed, Ts, 'zoh');
Gpid_d = c2d(Gpid, Ts, 'tustin');

fprintf('Sistema discretizado (Ts = %.5f s):\n', Ts);
disp(T_closed_d);

% Polos del sistema discreto
[zd, pd, kd] = zpkdata(T_closed_d, 'v');
fprintf('Polos discretos:\n');
for i = 1:length(pd)
    if imag(pd(i)) ~= 0
        fprintf('  p%d = %.4f %+.4fi  |  Magnitud: %.4f\n', ...
            i, real(pd(i)), imag(pd(i)), abs(pd(i)));
    else
        fprintf('  p%d = %.4f  |  Magnitud: %.4f\n', i, pd(i), abs(pd(i)));
    end
end

if all(abs(pd) < 1)
    fprintf('\n✓ Sistema discreto ESTABLE (|polos| < 1)\n\n');
else
    fprintf('\n✗ Sistema discreto INESTABLE\n\n');
end

%% 13. GUARDAR RESULTADOS
save('sistema_pid_puerta.mat', 'Gp', 'Gpid', 'T_closed', 'T_closed_d', ...
     'Kp', 'Ki', 'Kd', 'K', 'a', 'b', 'Ts', 'setpoints');

fprintf('═══ RESULTADOS GUARDADOS ═══\n');
fprintf('Archivo: sistema_pid_puerta.mat\n\n');

fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║            SIMULACIÓN COMPLETADA                       ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n');