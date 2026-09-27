%% ========================================================================
%  HEXÁPODO - Marcha trípode transmitida por la red Robotat
%  ------------------------------------------------------------------------
%  MATLAB --TCP:80--> ESP32 --UART 115200--> OpenCM9.04 --TTL 1Mbps--> 18x AX-12A
%
%  Este script fusiona:
%    * sim.m             
%  -> cinemática inversa del ciclo de marcha (tuya)
%    * Cinematica_Hex.m   -> esquema de conexión MATLAB->ESP32 (Luis Salazar)
%
%  DIFERENCIAS IMPORTANTES respecto al código de Luis:
%    1. La conexión TCP se abre UNA sola vez y se mantiene viva. Luis abría y
%       cerraba el socket por cada punto; eso sirve para una pose, no para
%       caminar.
%    2. No se envían valores crudos (0..1023) sino ÁNGULOS DEL MODELO en
%       decigrados. La calibración (offset/signo de cada servo) vive en la
%       OpenCM, que es donde ya la tienes en hexapodo_home.ino. Así hay una
%       sola fuente de verdad y no se desincronizan MATLAB y firmware.
%    3. El orden del arreglo es lineal: índice k -> servo con ID k.
%         q(1:3)   = pata 1 [coxa, fémur, tibia]  -> IDs 1,2,3
%         q(4:6)   = pata 2                       -> IDs 4,5,6   ... etc.
%
%  PROTOCOLO (una línea por trama, terminada en \n):
%       {"q":[q1,...,q18]}   ángulos del MODELO en decigrados (enteros)
%       {"c":"home"}         ir a la pose HOME
%       {"c":"relax"}        apagar torque
%       {"c":"torque"}       encender torque
%  El ESP32 responde "ok" por cada línea recibida.

%% ========================================================================

clc; clear; close all;


%% ------------------------- CONFIGURACIÓN --------------------------------
CFG.ip         = ['192.168.1.6' ...
    ''];  % IP que imprime el ESP32 en el monitor serie
CFG.port       = 80;
CFG.fs         = 40;               % Hz de envío (20 Hz = una trama cada 50 ms)
CFG.nCycles    = 8;                % ciclos de marcha (Inf = hasta Ctrl+C)
CFG.useAck     = true;             % esperar "ok" del ESP32 en cada trama
CFG.simOnly    = false;            % true = solo calcula/grafica, NO conecta

% --- SEGURIDAD: empezá chiquito ---
CFG.scale      = 1.0;             % 0..1 escala la amplitud del paso.
                                   % Primera prueba: 0.2-0.3 con el robot colgado.
CFG.activeLegs = [1 2 3 4 5 6];    % patas que se mueven; el resto se queda en HOME.
                                   % Para probar una sola pata: [2]

CFG.plotSim    = false;            % animar la marcha en MATLAB antes de enviar
CFG.homePause  = 2.5;              % s de espera después de mandar HOME

%% --------------------- MODELO DE LA PATA (de sim.m) ---------------------
L1 = 0.062732;   % COXA
L2 = 0.083;      % FÉMUR
L3 = 0.134390;   % TIBIA

s   = 'Rz(q1) Ty(L1) Rx(q2) Ty(L2) Rx(q3) Tz(L3)';
dh  = DHFactor(s);
leg = eval(dh.command('leg'));   %#ok<EVLEQ>

W = 0.150354;  L = 0.200354;  R = 0.085402;   % cuerpo (solo para graficar)
 
%% ------------------ CICLO DE MARCHA (idéntico a sim.m) ------------------
stride = 0.05;              % medio paso (m)
lift   = 0.05;              % altura de levantamiento (m)

qHome = [0 0.4 -0.3];       % configuración HOME del MODELO (rad)
pHome = transl(leg.fkine(qHome));
yy = pHome(2);
zd = pHome(3);
zu = zd - sign(zd)*lift;
xf =  stride;  xb = -stride;

segments = [ xf  yy  zd      % adelante, apoyado
             xb  yy  zd      % atrás, apoyado   (empuje)
             xb  yy  zu      % atrás, levantado
             xf  yy  zu ];   % adelante, levantado (regreso)

tseg_one = [ 0.25  1.0  0.25  0.5 ]';   % s por segmento -> ciclo = 2.0 s

dt_ik = 0.01;  tacc = 0.1;  nrep = 3;
x = mstraj(repmat(segments,nrep,1), [], repmat(tseg_one,nrep,1), ...
           segments(1,:), dt_ik, tacc);

nspc   = round(sum(tseg_one)/dt_ik);
xcycle = x(nspc+1 : 2*nspc, :);          % un ciclo limpio del centro

qcycle = leg.ikine( transl(xcycle), qHome, 'mask', [1 1 1 0 0 0] );
qcycle(:,1) = qcycle(:,1) - mean(qcycle(:,1));   % coxa oscila alrededor de HOME

fprintf('Ciclo resuelto: %d muestras a %.0f Hz (%.2f s por ciclo)\n', ...
        size(qcycle,1), 1/dt_ik, sum(tseg_one));

%% --------- DIEZMADO A LA TASA DE TRANSMISIÓN Y ARMADO DE TRAMAS ---------
% El IK se resuelve fino (100 Hz) pero se transmite a CFG.fs.
paso = max(1, round((1/CFG.fs)/dt_ik));
qtx  = qcycle(1:paso:end, :);
N    = size(qtx,1);

% Espejo CINEMÁTICO de la coxa: las patas 4-5-6 están montadas al otro lado,
% su eje +x local apunta al revés, así que hay que invertir el barrido para
% que las 6 patas empujen hacia el mismo lado.
% OJO: esto NO es lo mismo que JOINT_SIGN del firmware (que corrige el sentido
% MECÁNICO del servo). Si una pata camina al revés, invertí UNO de los dos,
% nunca los dos.
sgn = [ +1 +1 +1 -1 -1 -1 ];

% Marcha trípode: {1,3,5} en fase, {2,4,6} desfasadas medio ciclo.
off   = round(N/2);
phase = [ 0  off  0  off  0  off ];

% Ángulos HOME del modelo en grados (referencia para el firmware)
HOME_DEG_MODELO = rad2deg(qHome);

QDECI = zeros(N,18);
for k = 1:N
    for i = 1:6
        if ~ismember(i, CFG.activeLegs)
            ang = HOME_DEG_MODELO;              % pata quieta en HOME
        else
            kk = mod(k-1+phase(i), N) + 1;
            q  = qtx(kk,:);
            % Desviación respecto a HOME, escalada por seguridad
            dq = [ sgn(i)*q(1), q(2)-qHome(2), q(3)-qHome(3) ] * CFG.scale;
            ang = HOME_DEG_MODELO + rad2deg(dq);
        end
        QDECI(k, 3*(i-1)+(1:3)) = round(ang*10);   % decigrados enteros
    end
end

% Reporte de rangos (sirve para detectar movimientos absurdos ANTES de enviar)
dev = max(abs(QDECI - repmat(round(HOME_DEG_MODELO*10),1,6)), [], 1)/10;
fprintf('Desviación máx. respecto a HOME [deg]:\n');
for i = 1:6
    fprintf('  Pata %d -> coxa %5.1f | fémur %5.1f | tibia %5.1f\n', ...
            i, dev(3*(i-1)+1), dev(3*(i-1)+2), dev(3*(i-1)+3));
end

%% ------------------------ VISTA PREVIA (opcional) -----------------------
if CFG.plotSim
    legs(6) = SerialLink(leg,'name','leg6','base',SE3( L/2, W/2,0)*SE3.Rz(-pi/4));
    legs(5) = SerialLink(leg,'name','leg5','base',SE3(   0,   R,0));
    legs(4) = SerialLink(leg,'name','leg4','base',SE3(-L/2, W/2,0)*SE3.Rz(pi/4));
    legs(3) = SerialLink(leg,'name','leg3','base',SE3(-L/2,-W/2,0)*SE3.Rz(pi-pi/4));
    legs(2) = SerialLink(leg,'name','leg2','base',SE3(   0,  -R,0)*SE3.Rz(pi));
    legs(1) = SerialLink(leg,'name','leg1','base',SE3( L/2,-W/2,0)*SE3.Rz(pi+pi/4));

    figure; hold on; grid on; view(3); axis equal;
    axis([-0.4 0.4 -0.4 0.4 -0.3 0.3]);
    patch([L/2 L/2 -L/2 -L/2],[-W/2 W/2 W/2 -W/2],[0 0 0 0],'r','FaceAlpha',0.5);

    hipHome = [ -3*pi/4, pi, 3*pi/4, pi/4, 0, -pi/4 ];
    for k = 1:2:N
        for i = 1:6
            kk = mod(k-1+phase(i), N) + 1;
            qk = qtx(kk,:);
            qk(1) = hipHome(i) + sgn(i)*qk(1);
            legs(i).plot(qk,'nobase','noshadow','notiles','delay',0);
        end
        drawnow;
    end
end

if CFG.simOnly
    disp('CFG.simOnly = true -> no se conecta al ESP32. Fin.');
    return;
end

%% 
%% ---------------------------- CONEXIÓN ----------------------------------
esp32 = [];
try
    fprintf('Conectando a %s:%d ...\n', CFG.ip, CFG.port);
    esp32 = tcpclient(CFG.ip, CFG.port, 'Timeout', 5);
    configureTerminator(esp32, "LF");
    flush(esp32);

    saludo = readline(esp32);           % el ESP32 manda "READY"
    fprintf('ESP32: %s\n', strtrim(saludo));

    % --- 1. Pose HOME -----------------------------------------------------
    disp('Enviando HOME...');
    enviar(esp32, '{"c":"home"}', CFG);
    pause(CFG.homePause);

    input('Robot en HOME. ENTER para caminar (Ctrl+C para abortar)...','s');

    % --- 2. Streaming del ciclo de marcha ---------------------------------
    Ts = 1/CFG.fs;
    k = 1; ciclo = 0; tramas = 0;
    treloj = tic; tsig = 0;

    while ciclo < CFG.nCycles
        txt   = sprintf('%d,', QDECI(k,:));
        linea = ['{"q":[' txt(1:end-1) ']}'];

        enviar(esp32, linea, CFG);
        tramas = tramas + 1;

        k = k + 1;
        if k > N
            k = 1; ciclo = ciclo + 1;
            fprintf('  ciclo %d/%s completado\n', ciclo, string(CFG.nCycles));
        end

        tsig = tsig + Ts;
        espera = tsig - toc(treloj);
        if espera > 0, pause(espera); end
    end

    fprintf('Listo: %d tramas en %.1f s (%.1f Hz reales)\n', ...
            tramas, toc(treloj), tramas/toc(treloj));

    % --- 3. Regreso a HOME ------------------------------------------------
    disp('Regresando a HOME...');
    enviar(esp32, '{"c":"home"}', CFG);
    pause(CFG.homePause);

catch ME
    fprintf(2,'ERROR: %s\n', ME.message);
    if ~isempty(esp32) && isvalid(esp32)
        try
            writeline(esp32, '{"c":"home"}');   % intento de parada segura
        catch
        end
    end
end

if ~isempty(esp32)
    clear esp32;    % cierra el socket
end
disp('Conexión cerrada.');






%% ---------------------------- FUNCIONES ---------------------------------
function enviar(t, linea, CFG)
% Manda una línea y (opcionalmente) espera el "ok" del ESP32.
    writeline(t, linea);
    if CFG.useAck
        r = readline(t);
        if ~contains(r, "ok")
            warning('Respuesta inesperada del ESP32: "%s"', strtrim(r));
        end
    end
end
