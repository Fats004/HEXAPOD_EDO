% =========================================================================
% MT3005 - LABORATORIO 11: Control de robots móviles
% ADAPTACIÓN AL HEXAPOD EDO
% =========================================================================

%% Conexión al Robotat
robotat = robotat_connect();


%% Conexión al agente Hexapod
robot_no = 31;    % Número del agente. 

% Offset del marker, en GRADOS. 
offset = -0.4880;      

robot = robotat_hexapod_connect(robot_no);


%% Calibración del offset del marker

p1 = robotat_get_pose(robotat, robot_no, 'eulzyx');

T_cal = 10;                              % s caminando en recta
t0 = tic;
while toc(t0) < T_cal
    robotat_hexapod_advance_gait(robot, 'forward', 0.06);
    pause(0.03);
end
robotat_hexapod_force_stop(robot);
pause(1.0);

p2 = robotat_get_pose(robotat, robot_no, 'eulzyx');

rumbo  = atan2d(p2(2) - p1(2), p2(1) - p1(1));   % dirección real de avance
yaw_m  = atan2d(sind(p1(4)) + sind(p2(4)), cosd(p1(4)) + cosd(p2(4)));
offset = atan2d(sind(yaw_m - rumbo), cosd(yaw_m - rumbo));

disp(['Recorrido: ', num2str(1000*hypot(p2(1)-p1(1), p2(2)-p1(2)), '%.0f'), ...
      ' mm | offset = ', num2str(offset, '%.4f'), ' deg']);
% Si el recorrido sale menor a ~200 mm el rumbo no es confiable: repetir
% con más tiempo o revisar que las patas no estén patinando.

%% Visualización del marker del robot (si se requiere)
 robotat_trvisualize(robotat, 31);

%% Ejemplo de obtención de pose del robot

xi = robotat_get_pose(robotat, robot_no, 'eulzyx');
xpos = xi(1) * 1000; % en mm
ypos = xi(2) * 1000; % en mm
theta = atan2d(sind(xi(4) - offset), cosd(xi(4) - offset));



%% PUEDE TRABAJAR EL CONTROL AQUÍ

% =================================================================
% META
% =================================================================
meta_no = 75;     % Marcador que hace de meta.


fprintf('Limites del robot: v %.3f a %.3f m/s | w %.1f a %.1f deg/s\n', ...
        robot.vmin, robot.vmax, robot.wmin, robot.wmax);

% =================================================================
% SELECTOR DE CONTROLADOR Y PARÁMETROS (TUNEABLES)
% =================================================================
% Elige entre 'PID' (Acercamiento Exponencial) o 'LQR' (Feedback Linearization)
TIPO_CONTROLADOR = 'LQR';

% --- Parámetros para PID ---
K_rho   = 0.25;  
K_alpha = 0.8;    

% --- Parámetros para LQR ---
l_dist = 0.10;    % m. Punto de control por delante del centro del robot.
A = [0 0; 0 0];
B = [1 0; 0 1];
Q = [1 0; 0 1];
R_lqr = [10 0; 0 10];

if strcmp(TIPO_CONTROLADOR, 'LQR')
    disp('Calculando matriz de ganancias LQR...');
    K_ganancia_lqr = lqr(A, B, Q, R_lqr);
end

% =================================================================
% ARBITRAJE GIRO / AVANCE
% =================================================================
% El hexápodo no puede avanzar y girar en la misma trama, así que se
% alterna entre dos estados. 

ALINEADO_DEG    = 15;   % por debajo de esto, empieza a avanzar
DESALINEADO_DEG = 35;   % por encima de esto, vuelve a girar

% Sentido de giro. turn_gait recibe 'left' o 'right'.
SENTIDO_GIRO = +1;


TOL_LLEGADA = 150;   % mm

% =================================================================
% RITMO DEL LAZO
% =================================================================
% La marcha se ve entrecortada cuando el lazo no alcanza a mandar tramas
% con suficiente densidad. robotat_hexapod_step avanza la fase del ciclo
% en proporción al tiempo transcurrido entre llamadas, así que un lazo
% lento no camina más despacio: camina a saltos, porque cada trama mueve
% las patas un pedazo grande del ciclo de golpe.
%
% Hay dos cosas que frenan el lazo, y ninguna es el control:
%
%   robotat_get_pose  espera al servidor en incrementos de 100 ms. Si los
%                     datos no están listos, una sola lectura cuesta ese
%                     tiempo completo.
%   el ack del ESP32  con useAck activo, cada trama espera el "ok" de
%                     vuelta por WiFi antes de seguir.
%
% La solución es desacoplar: la pose se lee a ritmo de control (unas pocas
% veces por segundo, que es de sobra porque el robot gira a unos grados por
% segundo) y las tramas de marcha se mandan en cada vuelta, a ritmo de
% actuación, reusando la última consigna calculada.
T_POSE   = 0.20;   % s entre lecturas de pose     -> control a ~5 Hz
DT_LAZO  = 0.03;   % s entre tramas de marcha     -> actuación a ~30 Hz
USAR_ACK = false;  % false = no esperar el "ok" del ESP32 en cada trama

robot.gait.useAck = USAR_ACK;


girando = true;      % se arranca orientándose hacia la meta

% Consignas vigentes entre lecturas de pose
w_cmd     = robot.wmin;
v_cmd     = robot.vmin;
sentido_g = 'left';
sentido_a = 'forward';
llegado   = false;

primera     = true;
reloj_pose  = tic;
reloj_total = tic;
tramas      = 0;

% =================================================================
% REGISTRO PARA LA GRÁFICA
% =================================================================
% Se guarda una muestra por lectura de pose, no por trama: es el ritmo al
% que el control realmente decide algo. Se preasigna de más y al final se
% recorta, que sale mucho más barato que hacer crecer el arreglo dentro
% del lazo.
N_MAX     = 20000;
log_t     = zeros(N_MAX, 1);   % s desde el arranque
log_x     = zeros(N_MAX, 1);   % mm
log_y     = zeros(N_MAX, 1);   % mm
log_th    = zeros(N_MAX, 1);   % deg, ya con el offset aplicado
log_xd    = zeros(N_MAX, 1);   % mm, meta
log_yd    = zeros(N_MAX, 1);   % mm, meta
log_rho   = zeros(N_MAX, 1);   % mm
log_alpha = zeros(N_MAX, 1);   % deg
log_gira  = zeros(N_MAX, 1);   % 1 = girando, 0 = avanzando
log_v     = zeros(N_MAX, 1);   % m/s
log_w     = zeros(N_MAX, 1);   % deg/s
n_log     = 0;

disp(['Iniciando control de punto a punto usando [', TIPO_CONTROLADOR, '] ...']);

while true
    try
        % =============== CONTROL (a ritmo de T_POSE) ===============
        if primera || toc(reloj_pose) >= T_POSE
            reloj_pose = tic;
            primera    = false;

            % 1. Obtener pose actual del robot y de la meta SIMULTÁNEAMENTE
            poses = robotat_get_pose(robotat, [robot_no, meta_no], 'eulzyx');

            % Extraer pose del robot
            xpos = poses(1, 1) * 1000; % mm
            ypos = poses(1, 2) * 1000; % mm
            theta_deg = atan2d(sind(poses(1, 4) - offset), cosd(poses(1, 4) - offset));
            theta_rad = deg2rad(theta_deg);

            % Extraer pose de la meta dinámica
            xd = poses(2, 1) * 1000; % mm
            yd = poses(2, 2) * 1000; % mm

            % 2. Error de posición del centro del robot
            rho = sqrt((xd - xpos)^2 + (yd - ypos)^2);   % mm

            % 3. Condición de parada
            if rho < TOL_LLEGADA
                llegado = true;
            end

            % 4. CÁLCULO DE LEY DE CONTROL SEGÚN SELECTOR
            %    Ambos controladores entregan la pareja (v, w) del uniciclo:
            %    v en m/s y w en deg/s. Quién de los dos se ejecuta lo decide
            %    el arbitraje del paso 5.
            if strcmp(TIPO_CONTROLADOR, 'PID')
                % --- Controlador PID (Acercamiento Exponencial) ---
                alpha_deg = atan2d(yd - ypos, xd - xpos) - theta_deg;
                alpha_deg = atan2d(sind(alpha_deg), cosd(alpha_deg)); % Normalizar

                rho_m = rho / 1000;
                v = K_rho * (1 - exp(-rho_m));      % m/s
                w = K_alpha * alpha_deg;            % deg/s

            elseif strcmp(TIPO_CONTROLADOR, 'LQR')
                % --- Controlador LQR (Feedback Linearization) ---
                % Se controla un punto situado l_dist por delante del centro.
                % Todo en metros, que es la unidad en la que se sintonizó K.
                xp_m = xpos/1000 + l_dist * cos(theta_rad);
                yp_m = ypos/1000 + l_dist * sin(theta_rad);

                ex = xp_m - xd/1000;
                ey = yp_m - yd/1000;

                u  = -K_ganancia_lqr * [ex; ey];
                ux = u(1);
                uy = u(2);

                % Transformación inversa al uniciclo
                v     = cos(theta_rad)*ux + sin(theta_rad)*uy;                      % m/s
                w_rad = (-sin(theta_rad)/l_dist)*ux + (cos(theta_rad)/l_dist)*uy;   % rad/s
                w     = rad2deg(w_rad);                                             % deg/s

                % El arbitraje necesita saber qué tan desalineado está el robot,
                % dato que el LQR no entrega de forma explícita.
                alpha_deg = atan2d(yd - ypos, xd - xpos) - theta_deg;
                alpha_deg = atan2d(sind(alpha_deg), cosd(alpha_deg));
            else
                error('Tipo de controlador no válido. Use PID o LQR.');
            end

            % 5. ARBITRAJE: girar o avanzar, con histéresis
            %    (esto sustituye a la cinemática inversa del diferencial)
            if girando
                if abs(alpha_deg) < ALINEADO_DEG
                    girando = false;
                end
            else
                if abs(alpha_deg) > DESALINEADO_DEG
                    girando = true;
                end
            end

            % 6. Consignas que se van a repetir hasta la próxima lectura
            w_cmd = min(max(abs(w), robot.wmin), robot.wmax);
            v_cmd = min(max(abs(v), robot.vmin), robot.vmax);

            if SENTIDO_GIRO * alpha_deg >= 0
                sentido_g = 'left';
            else
                sentido_g = 'right';
            end

            if v >= 0
                sentido_a = 'forward';
            else
                sentido_a = 'backward';
            end

            % 7. Registro de la muestra
            if n_log < N_MAX
                n_log = n_log + 1;
                log_t(n_log)     = toc(reloj_total);
                log_x(n_log)     = xpos;
                log_y(n_log)     = ypos;
                log_th(n_log)    = theta_deg;
                log_xd(n_log)    = xd;
                log_yd(n_log)    = yd;
                log_rho(n_log)   = rho;
                log_alpha(n_log) = alpha_deg;
                log_gira(n_log)  = double(girando);
                log_v(n_log)     = v_cmd;
                log_w(n_log)     = w_cmd;
            end
        end

        if llegado
            disp('¡Meta alcanzada!');
            robotat_hexapod_force_stop(robot);
            break;
        end

        % =============== ACTUACIÓN (cada vuelta) ===============
        % Una trama por vuelta, con la consigna vigente. Es lo que mantiene
        % la marcha fluida entre lecturas de pose.
        if girando
            robotat_hexapod_turn_gait(robot, sentido_g, w_cmd);
        else
            robotat_hexapod_advance_gait(robot, sentido_a, v_cmd);
        end
        tramas = tramas + 1;

        % El ESP32 contesta "ok" aunque no lo estemos leyendo. Sin vaciar
        % el buffer de entrada cada tanto, esos bytes se acumulan durante
        % toda la corrida.
        if ~USAR_ACK && mod(tramas, 100) == 0
            flush(robot.tcpsock, "input");
        end

        pause(DT_LAZO);

    catch ME
        disp('Error en la comunicación o lectura de pose. Deteniendo...');
        disp(ME.message);
        robotat_hexapod_force_stop(robot);
        break;
    end
end

% Ritmo real conseguido. Por debajo de ~15 Hz la marcha se ve entrecortada:
% si sale bajo, subir T_POSE antes que bajar DT_LAZO.
fprintf('Ritmo real del lazo: %.1f Hz (%d tramas en %.1f s)\n', ...
        tramas / toc(reloj_total), tramas, toc(reloj_total));

%% Gráfica de la trayectoria y de las decisiones
% Cuatro vistas de la misma corrida: por dónde pasó el robot, qué tan
% lejos estuvo de la meta, qué tan desalineado iba, y cuál de las dos
% marchas eligió en cada momento. El color distingue la decisión: naranja
% mientras giraba sobre su eje, azul mientras avanzaba.

if n_log < 2
    disp('No hay suficientes muestras registradas para graficar.');
else

% Recorte del registro
n  = n_log;
tt = log_t(1:n);
xr = log_x(1:n);      yr = log_y(1:n);
hr = log_th(1:n);
xm = log_xd(1:n);     ym = log_yd(1:n);
rh = log_rho(1:n);
al = log_alpha(1:n);
gi = log_gira(1:n) > 0.5;
vc = log_v(1:n);      wc = log_w(1:n);

% Paleta. Tres colores fijos, verificados para que se distingan también
% con daltonismo y en impresión a escala de grises.
C_AVANZA = [0.055 0.486 0.753];   % azul
C_GIRA   = [0.851 0.416 0.059];   % naranja
C_META   = [0.545 0.294 0.788];   % morado
TINTA    = [0.25  0.25  0.25];
TINTA2   = [0.50  0.50  0.50];
REJILLA  = [0.86  0.86  0.86];

% Máscaras para partir el camino según la decisión. Se extienden una
% muestra a cada lado para que los tramos se toquen en los cambios de
% estado y la línea no quede cortada.
mg = gi   | [false; gi(1:end-1)]   | [gi(2:end); false];
ma = ~gi  | [false; ~gi(1:end-1)]  | [~gi(2:end); false];

fig = figure('Color', 'w', 'Position', [80 60 1180 860]);
TL  = tiledlayout(fig, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% ---------------------------------------------------------------- planta
ax1 = nexttile([2 2]);
hold(ax1, 'on'); box(ax1, 'on');

% Recorrido de la meta y círculo de tolerancia sobre su posición final
hM = plot(ax1, xm, ym, '--', 'Color', C_META, 'LineWidth', 1.2);
arc = linspace(0, 2*pi, 180);
plot(ax1, xm(end) + TOL_LLEGADA*cos(arc), ym(end) + TOL_LLEGADA*sin(arc), ...
     ':', 'Color', C_META, 'LineWidth', 1);
plot(ax1, xm(end), ym(end), 'o', 'MarkerSize', 9, ...
     'MarkerFaceColor', C_META, 'MarkerEdgeColor', 'w', 'LineWidth', 1);

% Camino del robot, separado por decisión
xa = xr; xa(~ma) = NaN;   ya = yr; ya(~ma) = NaN;
xg = xr; xg(~mg) = NaN;   yg = yr; yg(~mg) = NaN;
hA = plot(ax1, xa, ya, '-', 'Color', C_AVANZA, 'LineWidth', 2.2);
hG = plot(ax1, xg, yg, '-', 'Color', C_GIRA,   'LineWidth', 2.2);

% Orientación del robot cada tantas muestras
salto = max(1, round(n/25));
idx   = 1:salto:n;
esc_f = 0.07 * max([max(xr)-min(xr), max(yr)-min(yr), 400]);
quiver(ax1, xr(idx), yr(idx), esc_f*cosd(hr(idx)), esc_f*sind(hr(idx)), 0, ...
       'Color', TINTA2, 'LineWidth', 0.9, 'MaxHeadSize', 0.6);

% Inicio y fin
plot(ax1, xr(1), yr(1), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'w', ...
     'MarkerEdgeColor', TINTA, 'LineWidth', 1.6);
plot(ax1, xr(end), yr(end), 'o', 'MarkerSize', 8, 'MarkerFaceColor', TINTA, ...
     'MarkerEdgeColor', 'w', 'LineWidth', 1);
text(ax1, xr(1), yr(1), '  inicio', 'Color', TINTA, 'FontSize', 9);
text(ax1, xr(end), yr(end), '  fin', 'Color', TINTA, 'FontSize', 9);

axis(ax1, 'equal'); grid(ax1, 'on');
ax1.GridColor = REJILLA; ax1.GridAlpha = 1; ax1.Layer = 'top';
ax1.XColor = TINTA2; ax1.YColor = TINTA2;
xlabel(ax1, 'x [mm]'); ylabel(ax1, 'y [mm]');
title(ax1, 'Trayectoria en el plano del Robotat', 'Color', TINTA);
legend(ax1, [hA hG hM], {'avanzando', 'girando', 'meta (marcador)'}, ...
       'Location', 'best', 'Box', 'off', 'TextColor', TINTA);

% ------------------------------------------------------ distancia a meta
ax2 = nexttile; hold(ax2, 'on'); box(ax2, 'on');
plot(ax2, tt, rh, '-', 'Color', TINTA, 'LineWidth', 1.6);
plot(ax2, [tt(1) tt(end)], [TOL_LLEGADA TOL_LLEGADA], '--', ...
     'Color', C_META, 'LineWidth', 1);
text(ax2, tt(1), TOL_LLEGADA, ' tolerancia', 'Color', C_META, ...
     'FontSize', 8, 'VerticalAlignment', 'bottom');
grid(ax2, 'on'); ax2.GridColor = REJILLA; ax2.GridAlpha = 1;
ax2.XColor = TINTA2; ax2.YColor = TINTA2;
xlabel(ax2, 't [s]'); ylabel(ax2, '\rho [mm]');
title(ax2, 'Distancia a la meta', 'Color', TINTA);

% --------------------------------------------------- error de alineación
ax3 = nexttile; hold(ax3, 'on'); box(ax3, 'on');
ylim(ax3, [-190 190]);
yl = ylim(ax3);
k = 1;                     % sombrear los tramos en que estaba girando
while k <= n
    if gi(k)
        k2 = k;
        while k2 < n && gi(k2+1), k2 = k2 + 1; end
        patch(ax3, [tt(k) tt(k2) tt(k2) tt(k)], [yl(1) yl(1) yl(2) yl(2)], ...
              C_GIRA, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
        k = k2 + 1;
    else
        k = k + 1;
    end
end
plot(ax3, tt, al, '-', 'Color', TINTA, 'LineWidth', 1.6);
for sgn = [-1 1]
    plot(ax3, [tt(1) tt(end)], sgn*[1 1]*ALINEADO_DEG,    '-',  ...
         'Color', C_AVANZA, 'LineWidth', 1);
    plot(ax3, [tt(1) tt(end)], sgn*[1 1]*DESALINEADO_DEG, '--', ...
         'Color', C_GIRA,   'LineWidth', 1);
end
text(ax3, tt(end), ALINEADO_DEG, sprintf('%g ', ALINEADO_DEG), ...
     'Color', C_AVANZA, 'FontSize', 8, 'HorizontalAlignment', 'right', ...
     'VerticalAlignment', 'bottom');
text(ax3, tt(end), DESALINEADO_DEG, sprintf('%g ', DESALINEADO_DEG), ...
     'Color', C_GIRA, 'FontSize', 8, 'HorizontalAlignment', 'right', ...
     'VerticalAlignment', 'bottom');
grid(ax3, 'on'); ax3.GridColor = REJILLA; ax3.GridAlpha = 1; ax3.Layer = 'top';
ax3.XColor = TINTA2; ax3.YColor = TINTA2;
xlabel(ax3, 't [s]'); ylabel(ax3, '\alpha [deg]');
title(ax3, 'Desalineación y umbrales de histéresis (fondo = girando)', ...
      'Color', TINTA);

% ----------------------------------------------------- consigna de giro
ax4 = nexttile; hold(ax4, 'on'); box(ax4, 'on');
wplot = wc; wplot(~gi) = NaN;      % solo cuando de verdad se aplicó
plot(ax4, tt, wplot, '-', 'Color', C_GIRA, 'LineWidth', 1.8);
for yv = [robot.wmin robot.wmax]
    plot(ax4, [tt(1) tt(end)], [yv yv], ':', 'Color', TINTA2, 'LineWidth', 1);
end
ylim(ax4, [0 robot.wmax*1.15]);
grid(ax4, 'on'); ax4.GridColor = REJILLA; ax4.GridAlpha = 1;
ax4.XColor = TINTA2; ax4.YColor = TINTA2;
xlabel(ax4, 't [s]'); ylabel(ax4, '\omega [deg/s]');
title(ax4, 'Consigna de giro aplicada', 'Color', TINTA);

% --------------------------------------------------- consigna de avance
ax5 = nexttile; hold(ax5, 'on'); box(ax5, 'on');
vplot = vc; vplot(gi) = NaN;
plot(ax5, tt, vplot, '-', 'Color', C_AVANZA, 'LineWidth', 1.8);
for yv = [robot.vmin robot.vmax]
    plot(ax5, [tt(1) tt(end)], [yv yv], ':', 'Color', TINTA2, 'LineWidth', 1);
end
ylim(ax5, [0 robot.vmax*1.15]);
grid(ax5, 'on'); ax5.GridColor = REJILLA; ax5.GridAlpha = 1;
ax5.XColor = TINTA2; ax5.YColor = TINTA2;
xlabel(ax5, 't [s]'); ylabel(ax5, 'v [m/s]');
title(ax5, 'Consigna de avance aplicada', 'Color', TINTA);

linkaxes([ax2 ax3 ax4 ax5], 'x');

% Resumen numérico de la corrida
recorrido = sum(hypot(diff(xr), diff(yr)));
t_girando = sum(gi) / n * 100;
title(TL, sprintf(['HexaPod siguiendo el marcador %d  |  %s  |  %.1f s  |  ' ...
                   'recorrido %.0f mm  |  %.0f%% del tiempo girando  |  ' ...
                   '\\rho final %.0f mm'], ...
                  meta_no, TIPO_CONTROLADOR, tt(end), recorrido, ...
                  t_girando, rh(end)), ...
      'FontWeight', 'bold', 'Color', TINTA);

exportgraphics(fig, 'trayectoria_hexapod.png', 'Resolution', 200);
disp('Gráfica guardada en trayectoria_hexapod.png');

end

%% Desconexión del Robotat
robotat_disconnect(robotat);

%% Desconexión del Hexapod
robotat_hexapod_disconnect(robot);
