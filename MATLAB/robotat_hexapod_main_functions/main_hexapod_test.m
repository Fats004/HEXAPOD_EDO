% =========================================================================
% HEXÁPODO - PRUEBA DE LAS FUNCIONES DE MARCHA
% =========================================================================

%% ---------------------- 1. CONEXIÓN --------------------------------------
% En el laboratorio (IP sale del ID: 31 -> 192.168.50.231):
% hexa = robotat_hexapod_connect(31);
% En tu casa, con la IP que imprime el ESP32 en el monitor serie:
hexa = robotat_hexapod_connect(31);
%% ---------------------- 2. PARO DE EMERGENCIA ----------------------------
robotat_hexapod_force_stop(hexa);

%% ---------------------- 4. AVANCE POR TIEMPO ----------------------------
T = 8;                                   % segundos
v = 0.07;                                % m/sxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

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
% Predicción del modelo. Medí con cinta y compará: la diferencia es el
% patinaje de los pies, que es el dato interesante para la tesis.
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
ang = 180;                               % grados
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
% Avanzar y girar dentro del mismo lazo: la fase se conserva entre los dos,
% así que las patas no pegan un salto al cambiar de marcha.
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

%% ---------------------- 11. VELOCIDAD VARIABLE --------------------------
% Demuestra que la velocidad se puede cambiar EN CALIENTE, que es lo que
% va a hacer tu controlador. La fase no se reinicia.
t0 = tic;
while toc(t0) < 12
    v = 0.03 + 0.05*(0.5 + 0.5*sin(2*pi*toc(t0)/6));   % rampa suave
    robotat_hexapod_advance_gait(hexa, 'forward', v);
    pause(0.03);
end
robotat_hexapod_force_stop(hexa);

%% ---------------------- 12. RITMO DEL LAZO ------------------------------
% Verifica que tu lazo llama lo suficientemente rápido. Por debajo de
% ~15 Hz la marcha se ve entrecortada.
t0 = tic;
while toc(t0) < 5
    info = robotat_hexapod_advance_gait(hexa, 'forward', 0.05);
    pause(0.03);
end
robotat_hexapod_force_stop(hexa);
fprintf('Ritmo del lazo: %.1f Hz\n', info.fs_real);

%% ---------------------- 13. BAILE: UN PASO A LA VEZ ---------------------
% El baile no desplaza al robot, mueve el CUERPO sobre las patas apoyadas.
% Aun así conviene la primera vez dejarle espacio libre alrededor y verlo
% en el suelo, no colgado: los pasos se apoyan en que los pies no resbalan.
%
%   robotat_hexapod_dance(hexa, paso, bpm, compases)
%
%     paso      'rebote'   sube y baja el cuerpo a tiempo
%               'balanceo' se mece de lado a lado
%               'cabeceo'  cabecea adelante y atrás
%               'twist'    barre las coxas sin avance neto
%               'ola'      cada pata se recoge por turno
%               'saludo'   levanta la pata delantera derecha y la ondea
%     bpm       tempo. 80 es lento y se ve bien; 140 se ve nervioso.
%     compases  cuántos compases de cuatro tiempos dura el paso.
%
% Cada llamada termina mandando al robot a HOME, así que se pueden
% encadenar sin preocuparse por dónde quedó el anterior.

robotat_hexapod_dance(hexa, 'rebote',   100, 2);
robotat_hexapod_dance(hexa, 'balanceo', 100, 2);
robotat_hexapod_dance(hexa, 'cabeceo',   90, 2);
robotat_hexapod_dance(hexa, 'twist',    110, 2);
robotat_hexapod_dance(hexa, 'ola',       80, 2);
robotat_hexapod_dance(hexa, 'saludo',   100, 2);

%% ---------------------- 14. BAILE: COREOGRAFÍA --------------------------
% Sin argumentos: los seis pasos en orden, a 100 bpm y 4 compases cada uno.
robotat_hexapod_dance(hexa);

% Coreografía propia: se pasa un cell con el orden que quieras. Se pueden
% repetir pasos, que es lo que hace que parezca una rutina y no una lista.
mezcla = {'rebote', 'twist', 'rebote', 'ola', 'balanceo', 'saludo'};
info   = robotat_hexapod_dance(hexa, mezcla, 120, 2);
fprintf('Baile: %d tramas en %.1f s (%.1f Hz reales)\n', ...
        info.tramas, info.t_real, info.fs_real);

% La amplitud sale de la escala del robot, la misma que usa la marcha:
%     hexa.gait.escala = 0.6;   % más contenido
%     hexa.gait.escala = 1.0;   % amplitud completa
% Con la escala en 1.0 el cuerpo recorre unos 30 mm en el rebote y el
% balanceo, que es lo más que aguanta la linealización de la pata.

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
