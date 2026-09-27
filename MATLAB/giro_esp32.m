

%% ========================================================================
%  HEXÁPODO - GIRO SOBRE SU PROPIO EJE transmitido por la red Robotat
%  ------------------------------------------------------------------------
%  MATLAB --TCP:80--> ESP32 --UART 115200--> OpenCM9.04 --TTL 1Mbps--> 18x AX-12A
%
%  Gemelo de caminata_esp32.m. Mismo protocolo, mismo diezmado, misma
%  conexión. La cinemática del ciclo es IDÉNTICA (el mismo rectángulo del
%  pie resuelto con la misma ikine); lo único que cambia es hacia dónde
%  empuja cada pata durante su fase de apoyo:
%
%     avance -> sgn = [ +1 +1 +1 -1 -1 -1 ]
%               las patas de un lado y del otro empujan en sentidos
%               opuestos en su marco local -> el cuerpo se traslada.
%
%     giro   -> sgn = [ +1 +1 +1 +1 +1 +1 ]
%               las 6 empujan en el MISMO sentido angular -> las fuerzas
%               se cancelan en traslación y solo queda par -> el cuerpo
%               gira sobre su propio eje.
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
CFG.ip         = '192.168.1.6';  % IP que imprime el ESP32 en el monitor serie
CFG.port       = 80;
CFG.fs         = 40;               % Hz de envío
CFG.nCycles    = 8;                % ciclos de giro (Inf = hasta Ctrl+C)
CFG.useAck     = true;             % esperar "ok" del ESP32 en cada trama
CFG.simOnly    = false;            % true = solo calcula/grafica, NO conecta

% --- SENTIDO DEL GIRO ---
CFG.dir        = +1;               % +1 = un sentido, -1 = el contrario.
                                   % Cuál es horario depende del montaje;
                                   % se determina en la primera prueba.

% --- SEGURIDAD: empezá chiquito ---
CFG.scale      = 0.70;              % 0..1 escala la amplitud del barrido.
                                   % Primera prueba: 0.2-0.3 con el robot colgado.
CFG.activeLegs = [1 2 3 4 5 6];    % patas que se mueven; el resto se queda en HOME.

% Compensación radial: las patas medias están más cerca del centro que las
% de esquina, así que para el MISMO giro del cuerpo necesitan un barrido de
% coxa menor. Sin esto las 6 patas piden giros distintos y se arrastran.
CFG.rescaleRadial = true;

CFG.plotSim    = false;            % animar el giro en MATLAB antes de enviar
CFG.homePause  = 2.5;              % s de espera después de mandar HOME

%% --------------------- MODELO DE LA PATA (de sim.m) ---------------------
L1 = 0.062732;   % COXA
L2 = 0.083;      % FÉMUR
L3 = 0.134390;   % TIBIA

s   = 'Rz(q1) Ty(L1) Rx(q2) Ty(L2) Rx(q3) Tz(L3)';
dh  = DHFactor(s);
leg = eval(dh.command('leg'));   %#ok<EVLEQ>

W = 0.150354;  L = 0.200354;  R = 0.085402;   % cuerpo

%% ---------------- CICLO DE GIRO (idéntico a sim_giro.m) -----------------
stride = 0.05;              % medio barrido (m)
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
paso = max(1, round((1/CFG.fs)/dt_ik));
qtx  = qcycle(1:paso:end, :);
N    = size(qtx,1);

% ------------------------------------------------------------------------
% ESTA ES LA ÚNICA DIFERENCIA CINEMÁTICA CON caminata_esp32.m
% ------------------------------------------------------------------------
sgn = CFG.dir * [ +1 +1 +1 +1 +1 +1 ];

% Marcha trípode: {1,3,5} en fase, {2,4,6} desfasadas medio ciclo.
off   = round(N/2);
phase = [ 0  off  0  off  0  off ];

% ---- Geometría de los pies (para el ajuste radial y las estimaciones) ---
hipXY = [  L/2 -W/2 ;  0 -R ; -L/2 -W/2 ; -L/2  W/2 ;  0  R ;  L/2  W/2 ];
azLeg = [ -pi/4 ; -pi/2 ; -3*pi/4 ; 3*pi/4 ; pi/2 ; pi/4 ];  % hacia afuera

pieXY = [ hipXY(:,1) + yy*cos(azLeg), hipXY(:,2) + yy*sin(azLeg) ];
rPie  = hypot(pieXY(:,1), pieXY(:,2));      % radio de cada pie al centro

if CFG.rescaleRadial
    kleg = (rPie / mean(rPie))';            % ganancia de coxa por pata
else
    kleg = ones(1,6);
end

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
            dq = [ sgn(i)*kleg(i)*q(1), q(2)-qHome(2), q(3)-qHome(3) ] * CFG.scale;
            ang = HOME_DEG_MODELO + rad2deg(dq);
        end
        QDECI(k, 3*(i-1)+(1:3)) = round(ang*10);   % decigrados enteros
    end
end

% Reporte de rangos
dev = max(abs(QDECI - repmat(round(HOME_DEG_MODELO*10),1,6)), [], 1)/10;
fprintf('Desviación máx. respecto a HOME [deg]:\n');
for i = 1:6
    fprintf('  Pata %d -> coxa %5.1f | fémur %5.1f | tibia %5.1f\n', ...
            i, dev(3*(i-1)+1), dev(3*(i-1)+2), dev(3*(i-1)+3));
end

%% ---------------------- ESTIMACIÓN DEL GIRO ESPERADO --------------------
% Cada fase de apoyo barre el pie un arco de 2*stride sobre un círculo de
% radio rPie. El cuerpo gira ese arco dividido entre el radio. En marcha
% trípode hay DOS fases de apoyo por ciclo.
sweep    = max(abs(qtx(:,1)));                       % barrido de coxa (rad)
arco     = 2*stride*CFG.scale;                       % arco por apoyo (m)
girApoyo = arco / mean(rPie);                        % rad por fase de apoyo
girCiclo = 2*girApoyo;                               % rad por ciclo completo
Tciclo   = N / CFG.fs;                               % s por ciclo (real)

fprintf('\n--- Giro esperado ---\n');
fprintf('  Barrido de coxa      : %.2f deg (+-)\n', rad2deg(sweep)*CFG.scale);
fprintf('  Radio medio del pie  : %.1f mm\n', 1000*mean(rPie));
fprintf('  Giro por apoyo       : %.2f deg\n', rad2deg(girApoyo));
fprintf('  Giro por ciclo       : %.2f deg\n', rad2deg(girCiclo));
fprintf('  Duración del ciclo   : %.3f s (%d tramas a %.0f Hz)\n', Tciclo, N, CFG.fs);
fprintf('  Velocidad angular    : %.1f deg/s\n', rad2deg(girCiclo)/Tciclo);
fprintf('  Vuelta completa      : %.1f ciclos = %.1f s\n', ...
        360/rad2deg(girCiclo), 360/rad2deg(girCiclo)*Tciclo);

% --- Holgura entre patas vecinas ---------------------------------------
% Las patas vecinas más cercanas están a 45 deg. Como los dos trípodes van
% en antifase, en el peor instante esa separación se cierra 2*barrido.
holgura = 45 - 2*rad2deg(sweep)*CFG.scale*max(kleg);
fprintf('  Holgura mín. vecinas : %.1f deg (~%.0f mm entre pies)\n', ...
        holgura, 1000*deg2rad(holgura)*mean(rPie));
if holgura < 8
    fprintf(2,'  ATENCIÓN: holgura muy baja, riesgo de choque. Bajá CFG.scale o stride.\n');
end
fprintf('\n');

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
            qk(1) = hipHome(i) + sgn(i)*kleg(i)*qk(1)*CFG.scale;
            legs(i).plot(qk,'nobase','noshadow','notiles','delay',0);
        end
        drawnow;
    end
end

if CFG.simOnly
    disp('CFG.simOnly = true -> no se conecta al ESP32. Fin.');
    return;
end

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

    input('Robot en HOME. ENTER para girar (Ctrl+C para abortar)...','s');

    % --- 2. Streaming del ciclo de giro -----------------------------------
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
            fprintf('  ciclo %d/%s completado (~%.0f deg acumulados)\n', ...
                    ciclo, string(CFG.nCycles), ciclo*rad2deg(girCiclo));
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