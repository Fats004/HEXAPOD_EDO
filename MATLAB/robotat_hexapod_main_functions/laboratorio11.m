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
% PARÁMETROS DEL CONTROLADOR LQR (Feedback Linearization)
% =================================================================
l_dist = 0.10;    % m. Punto de control por delante del centro del robot.
A = [0 0; 0 0];
B = [1 0; 0 1];
Q = [1 0; 0 1];
R_lqr = [10 0; 0 10];

disp('Calculando matriz de ganancias LQR...');
K_ganancia_lqr = lqr(A, B, Q, R_lqr);

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

% La pose se lee a ritmo de control (unas pocas
% veces por segundo, que es de sobra porque el robot gira a unos grados por
% segundo) y las tramas de marcha se mandan en cada vuelta.

T_POSE   = 0.20;   % s entre lecturas de pose     -> control a ~5 Hz
DT_LAZO  = 0.03;   % s entre tramas de marcha     -> actuación a ~30 Hz
USAR_ACK = false;  % false = no esperar el "ok" del ESP32 en cada trama

robot.gait.useAck = USAR_ACK;


girando = true;      % se arranca orientándose hacia la meta


w_cmd     = robot.wmin;
v_cmd     = robot.vmin;
sentido_g = 'left';
sentido_a = 'forward';
llegado   = false;

primera     = true;
reloj_pose  = tic;
tramas      = 0;

disp('Iniciando control de punto a punto usando LQR ...');

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

            % Extraer pose de la meta 
            xd = poses(2, 1) * 1000; % mm
            yd = poses(2, 2) * 1000; % mm

            % 2. Error de posición del centro del robot
            rho = sqrt((xd - xpos)^2 + (yd - ypos)^2);   % mm

            % 3. Condición de parada
            if rho < TOL_LLEGADA
                llegado = true;
            end

            % 4. LEY DE CONTROL LQR

            %    Se controla un punto situado l_dist por delante del centro.
            %    Todo en metros, que es la unidad en la que se sintonizó K.
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


            alpha_deg = atan2d(yd - ypos, xd - xpos) - theta_deg;
            alpha_deg = atan2d(sind(alpha_deg), cosd(alpha_deg));

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
        end

        if llegado
            disp('¡Meta alcanzada!');
            robotat_hexapod_force_stop(robot);
            break;
        end

        if girando
            robotat_hexapod_turn_gait(robot, sentido_g, w_cmd);
        else
            robotat_hexapod_advance_gait(robot, sentido_a, v_cmd);
        end
        tramas = tramas + 1;

        % El ESP32 contesta "ok" aunque no lo estemos leyendo. 
        
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


%% Desconexión del Robotat
robotat_disconnect(robotat);

%% Desconexión del Hexapod
robotat_hexapod_disconnect(robot);
