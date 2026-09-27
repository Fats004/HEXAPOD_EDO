function info = robotat_hexapod_dance(robot, rutina, bpm, compases)
%ROBOTAT_HEXAPOD_DANCE  Rutina de baile para el HexaPod.
%
%   robotat_hexapod_dance(robot)
%       Coreografía completa a 100 bpm.
%
%   robotat_hexapod_dance(robot, 'rebote', 120, 4)
%       Un solo paso, a 120 bpm, durante 4 compases.
%
%   info = robotat_hexapod_dance(...)
%       Devuelve tramas enviadas, duración y ritmo real.
%
%   PASOS DISPONIBLES
%       'rebote'    las seis patas se extienden y recogen a la vez; el
%                   cuerpo sube y baja a tiempo.
%       'balanceo'  el lado A extiende mientras el B recoge; el cuerpo se
%                   mece de lado a lado.
%       'cabeceo'   las patas delanteras extienden mientras las traseras
%                   recogen; el cuerpo cabecea.
%       'twist'     las coxas barren a un lado y al otro sin avance neto.
%       'ola'       cada pata se recoge por turno dando la vuelta al cuerpo.
%       'saludo'    la pata delantera derecha se levanta y ondea.
%       'completa'  los seis anteriores, en secuencia.
%
%   -----------------------------------------------------------------------
%   CÓMO FUNCIONA
%   Las funciones de marcha resuelven trayectorias del PIE; para bailar
%   interesa mover el CUERPO, así que esta función manda las tramas
%   directamente. Cada trama es la misma que usa robotat_hexapod_step:
%   dieciocho ángulos del modelo en decigrados, con la calibración de cada
%   pata resolviéndose en la OpenCM9.04.
%
%   La amplitud se expresa en MILÍMETROS de movimiento del cuerpo, no en
%   grados de articulación, porque es lo que se puede ver y medir. La
%   conversión sale de derivar la cinemática de la pata alrededor de HOME:
%   para subir el cuerpo 1 mm con el pie apoyado y sin arrastrarlo de lado,
%   el fémur sube 0.719 grados y la tibia baja 0.893. Esa pareja es la
%   dirección EXTENDER de abajo. La linealización aguanta bien hasta unos
%   30 mm (a 30 mm pedidos el cuerpo sube 26.5 y el pie se corre 5 mm), que
%   es de sobra para bailar.
%
%   El robot nunca levanta más de una pata a la vez, así que siempre quedan
%   cinco apoyadas y la plataforma no pierde estabilidad.

    if nargin < 2 || isempty(rutina),   rutina   = 'completa'; end
    if nargin < 3 || isempty(bpm),      bpm      = 100;        end
    if nargin < 4 || isempty(compases), compases = 4;          end

    if ~isfield(robot, 'tcpsock') || ~isvalid(robot.tcpsock)
        error('El robot no está conectado.');
    end

    % ------------------------- Geometría del cuerpo -----------------------
    % Posición de cada pata, tomada de hipXY en robotat_hexapod_connect.
    % Columna 1: +1 delantera, 0 media, -1 trasera.
    % Columna 2: -1 lado A (izquierda), +1 lado B (derecha).
    POS = [ +1 -1 ;    % pata 1   delantera  lado A
             0 -1 ;    % pata 2   media      lado A
            -1 -1 ;    % pata 3   trasera    lado A
            -1 +1 ;    % pata 4   trasera    lado B
             0 +1 ;    % pata 5   media      lado B
            +1 +1 ];   % pata 6   delantera  lado B

    % Las patas del lado B están montadas en espejo: para que las seis
    % coxas barran hacia el mismo lado del cuerpo hay que invertirlas. Es
    % el mismo vector sgn que usa robotat_hexapod_advance_gait.
    ESPEJO_COXA = [ +1 +1 +1 -1 -1 -1 ];

    % Dirección "extender la pata" en grados por milímetro de cuerpo.
    % [fémur, tibia]. Positivo = el cuerpo sube.
    EXTENDER = [ +0.7190, -0.8928 ];

    % ------------------------- Límites de seguridad -----------------------
    % El firmware recorta la desviación respecto a HOME en 58/73/73 grados.
    % Esto se queda bien adentro; además la escala del robot sigue valiendo.
    LIM_DEG   = [ 25, 35, 40 ];        % coxa, fémur, tibia
    ALT_MAX   = 30;                    % mm de recorrido vertical del cuerpo
    esc       = robot.gait.escala;

    HOME_DEG  = rad2deg(robot.gait.qHome);
    FS        = 30;                    % Hz de envío
    Ts        = 1 / FS;
    Tcompas   = 4 * 60 / bpm;          % s por compás de cuatro tiempos

    % --------------------------- Coreografía ------------------------------
    if iscell(rutina)
        pasos = rutina;
    elseif strcmpi(rutina, 'completa')
        pasos = {'rebote', 'balanceo', 'twist', 'cabeceo', 'ola', 'saludo'};
    else
        pasos = {rutina};
    end

    fprintf('Bailando: %s | %d bpm | %.1f s por compás\n', ...
            strjoin(pasos, ' - '), bpm, Tcompas);

    % Si se aborta con Ctrl+C, el robot regresa a HOME en vez de quedarse
    % en una postura a medio paso.
    limpieza = onCleanup(@() robotat_hexapod_force_stop(robot));  %#ok<NASGU>

    tramas  = 0;
    treloj  = tic;

    for p = 1:numel(pasos)
        paso  = lower(pasos{p});
        Tpaso = compases * Tcompas;
        fprintf('  %s\n', paso);

        tpaso = tic;
        tsig  = 0;
        while toc(tpaso) < Tpaso
            t   = toc(tpaso);
            th  = 2*pi * t / Tcompas;        % fase del compás, en radianes
            d   = zeros(6, 3);               % desviación respecto a HOME

            switch paso
                case 'rebote'
                    % Las seis patas a la vez, dos rebotes por compás.
                    a = ALT_MAX * 0.7 * sin(2*th);
                    d(:, 2:3) = repmat(a * EXTENDER, 6, 1);

                case 'balanceo'
                    % Roll: un lado extiende mientras el otro recoge.
                    a = ALT_MAX * sin(th);
                    d(:, 2:3) = (-POS(:, 2) * a) * EXTENDER;

                case 'cabeceo'
                    % Pitch: delanteras contra traseras. Las medias no se
                    % mueven, que es justo lo que hace POS(:,1) = 0.
                    a = ALT_MAX * sin(th);
                    d(:, 2:3) = (POS(:, 1) * a) * EXTENDER;

                case 'twist'
                    % Las coxas barren a un lado y al otro. Con el espejo
                    % aplicado el cuerpo gira, y como es un seno alrededor
                    % de cero no hay avance neto: se queda en el sitio.
                    a = 18 * sin(2*th);
                    d(:, 1) = ESPEJO_COXA(:) * a;

                case 'ola'
                    % Cada pata se recoge por turno, dando una vuelta
                    % completa al cuerpo por compás. El orden es el físico
                    % 1-2-3-4-5-6, así que la ola recorre un lado y regresa
                    % por el otro.
                    %
                    % El pulso ocupa exactamente un sexto del ciclo, de modo
                    % que en todo momento hay UNA sola pata levantada y
                    % cinco apoyadas. Con un medio seno, que es lo natural
                    % de escribir, se solapan hasta tres patas en el aire y
                    % el robot se queda sin triángulo de apoyo.
                    for i = 1:6
                        u = mod(th/(2*pi) - (i-1)/6, 1);   % 0..1 en el ciclo
                        if u < 1/6
                            s = 0.5 * (1 - cos(2*pi * 6*u));   % sube y baja
                        else
                            s = 0;
                        end
                        d(i, 2:3) = (-ALT_MAX * 0.9 * s) * EXTENDER;
                        d(i, 1)   = ESPEJO_COXA(i) * 10 * s;
                    end

                case 'saludo'
                    % La pata 6 (delantera del lado B) se recoge y ondea
                    % con la coxa. Las otras cinco bajan un poco para
                    % compensar el peso que se corre hacia atrás.
                    d(:, 2:3) = repmat((-4) * EXTENDER, 6, 1);
                    d(6, 2:3) = (-ALT_MAX * 1.0) * EXTENDER;
                    d(6, 1)   = ESPEJO_COXA(6) * 22 * sin(3*th);

                otherwise
                    error('Paso de baile desconocido: %s', paso);
            end

            % Escala de seguridad del robot y recorte por articulación
            d = d * esc;
            for j = 1:3
                d(:, j) = max(min(d(:, j), LIM_DEG(j)), -LIM_DEG(j));
            end

            enviar_pose(robot, HOME_DEG, d);
            tramas = tramas + 1;

            if ~robot.gait.useAck && mod(tramas, 100) == 0
                flush(robot.tcpsock, "input");
            end

            tsig = tsig + Ts;
            espera = tsig - toc(tpaso);
            if espera > 0, pause(espera); end
        end
    end

    robotat_hexapod_force_stop(robot);

    t_real = toc(treloj);
    info.tramas  = tramas;
    info.t_real  = t_real;
    info.fs_real = tramas / t_real;
    info.pasos   = pasos;
    fprintf('Listo: %d tramas en %.1f s (%.1f Hz reales)\n', ...
            tramas, t_real, info.fs_real);
end


% =========================================================================
function enviar_pose(robot, HOME_DEG, d)
% Arma y manda una trama de dieciocho ángulos del modelo, en decigrados.
% d es 6x3: desviación respecto a HOME de [coxa, fémur, tibia] por pata.

    q = zeros(1, 18);
    for i = 1:6
        q(3*(i-1) + (1:3)) = round( (HOME_DEG + d(i, :)) * 10 );
    end

    txt = sprintf('%d,', q);
    writeline(robot.tcpsock, ['{"q":[' txt(1:end-1) ']}']);

    if robot.gait.useAck
        r = readline(robot.tcpsock);
        if ismissing(r)
            warning('El ESP32 no respondió a la trama.');
        end
    end
end
