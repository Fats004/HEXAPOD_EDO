% =========================================================================
% HEXÁPODO - PRUEBA DE LAS FUNCIONES DE MARCHA
% =========================================================================

%% ---------------------- 1. CONEXIÓN --------------------------------------
% En el laboratorio (IP sale del ID: 31 -> 192.168.50.231):
% hexa = robotat_hexapod_connect(31);
% Fuera del lab, con la IP que imprime el ESP32 en el monitor serie:
hexa = robotat_hexapod_connect(31);

%% ---------------------- 2. PARO DE EMERGENCIA ----------------------------
robotat_hexapod_force_stop(hexa);

%% ---------------------- 4. AVANCE POR TIEMPO ----------------------------
T = 30;                                   % segundos
v = 0.07;                                % m/s

t0 = tic;
while toc(t0) < T
    robotat_hexapod_advance_gait(hexa, 'forward', v);
    pause(0.03);
end
robotat_hexapod_force_stop(hexa);
fprintf('Recorrido predicho: %.0f mm\n', 1000*v*T);

%% ---------------------- 5. RETROCESO ------------------------------------
t0 = tic;
while toc(t0) < 8
    robotat_hexapod_advance_gait(hexa, 'backward', 0.05);
    pause(0.03);
end
robotat_hexapod_force_stop(hexa);

%% ---------------------- 6. AMPLITUD COMPLETA ----------------------------
hexa.gait.escala = 1.0;
hexa = actualizar_limites(hexa);

%% ---------------------- 7. AVANCE POR DISTANCIA -------------------------
d = 0.80;                                % metros
v = 0.08;                                % m/s

t0 = tic;
while toc(t0) < d/v
    robotat_hexapod_advance_gait(hexa, 'forward', v);
    pause(0.03);
end
robotat_hexapod_force_stop(hexa);
fprintf('Predicho %.0f mm. Medí con cinta.\n', 1000*d);

%% ---------------------- 8. GIRO POR ÁNGULO ------------------------------
ang = 360;                               % grados
w   = 21;                                % deg/s

t0 = tic;
while toc(t0) < ang/w
    robotat_hexapod_turn_gait(hexa, 'r', w);
    pause(0.03);
end
robotat_hexapod_force_stop(hexa);
fprintf('Predicho %.0f deg. Medí con transportador.\n', ang);

%% ---------------------- 9. GIRO A LA DERECHA ----------------------------
t0 = tic;
while toc(t0) < 6
    robotat_hexapod_turn_gait(hexa, 'right', 20);
    pause(0.03);
end
robotat_hexapod_force_stop(hexa);

%% ---------------------- 10. CUADRADO ------------------------------------
for lado = 1:4
    t0 = tic;
    while toc(t0) < 0.50/0.07
        robotat_hexapod_advance_gait(hexa, 'forward', 0.08);
        pause(0.03);
    end

    t0 = tic;
    while toc(t0) < 90/20
        robotat_hexapod_turn_gait(hexa, 'left', 20);
        pause(0.03);
    end
end
robotat_hexapod_force_stop(hexa);

%% ---------------------- 14. BAILE: COREOGRAFÍA --------------------------

mezcla = {'rebote', 'twist', 'rebote', 'ola', 'balanceo', 'saludo'};
info   = robotat_hexapod_dance(hexa, mezcla, 140, 2);

%% ---------------------- 15. DESCONEXIÓN ---------------------------------
robotat_hexapod_disconnect(hexa);


%% =========================================================================
function robot = actualizar_limites(robot)
% Recalcula los topes de velocidad después de cambiar robot.gait.escala.
    Dciclo = 2 * (2*robot.gait.stride) * robot.gait.escala;
    Gciclo = Dciclo / robot.gait.rMedio;

    robot.vmax = Dciclo / robot.gait.Tmin;
    robot.vmin = Dciclo / robot.gait.Tmax;
    robot.wmax = rad2deg(Gciclo) / robot.gait.Tmin;
    robot.wmin = rad2deg(Gciclo) / robot.gait.Tmax;

    fprintf('Escala %.2f -> v: %.1f a %.1f cm/s | w: %.1f a %.1f deg/s\n', ...
            robot.gait.escala, 100*robot.vmin, 100*robot.vmax, ...
            robot.wmin, robot.wmax);
end
