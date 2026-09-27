function robot = robotat_hexapod_connect(agent_id, ip_manual)
% 
%   robot = robotat_hexapod_connect(31, '192.168.1.6')
%       Cualquier otra red. La IP la imprime el ESP32 en el
%       monitor serie al arrancar. 
%
%   robot = robotat_hexapod_connect(31)
%       Red del laboratorio.
%           ID 31 -> 192.168.50.231
%           ID 36 -> 192.168.50.236
%
%   IDs permitidos: 31 - 36.
%
%   Además de conectar, esta función precalcula UNA sola vez la cinemática
%   inversa del ciclo de marcha y la guarda en robot.gait, para que 
%   advance_gait y turn_gait no tengan que resolverla cada vez.

    %% ---------------------- Validación del ID ---------------------------
    if numel(agent_id) ~= 1
        error('Can only pair with a single hexapod agent.');
    end
    agent_id = round(agent_id);
    if (agent_id < 31) || (agent_id > 36)
        error('Invalid agent ID. Allowed IDs: 31 - 36.');
    end

    robot.id = agent_id;

    %% ------------------------- Dirección IP -----------------------------
    if nargin < 2 || isempty(ip_manual)
        robot.ip  = ['192.168.50.2', num2str(agent_id)];   % red del Robotat
        robot.red = 'robotat';
    else
        robot.ip  = char(ip_manual);                       % otra red
        robot.red = 'manual';
    end
    robot.port = 80;

    %% ------------------- Modelo cinemático de la pata --------------------
    L1 = 0.062732;   % COXA
    L2 = 0.083;      % FEMUR
    L3 = 0.134390;   % TIBIA

    % BASE
    W  = 0.150354;  L = 0.200354;  R = 0.085402;
    
    % Derivación cinemática
    s   = 'Rz(q1) Ty(L1) Rx(q2) Ty(L2) Rx(q3) Tz(L3)';
    dh  = DHFactor(s);
    leg = eval(dh.command('leg'));   

    %% ------------------ Ciclo de marcha (una sola vez) -------------------
    stride = 0.04;              % zancada = 2*stride (m)
    lift   = 0.05;              % levantamiento del pie (m)
    qHome  = [0 0.4 -0.3];      % HOME (rad)

    pHome = transl(leg.fkine(qHome));
    yy = pHome(2);                      
    zd = pHome(3);                      % pie apoyado
    zu = zd - sign(zd)*lift;            % pie levantado

    segments = [  stride  yy  zd        % adelante, apoyado
                 -stride  yy  zd        % atrás, apoyado (empuje)
                 -stride  yy  zu        % atrás, levantado
                  stride  yy  zu ];     % adelante, levantado (regreso)

    tseg  = [0.25  1.0  0.25  0.5]';    % ciclo = 2.0 s
    dt_ik = 0.01;  tacc = 0.1;  nrep = 3;

    fprintf('Resolviendo cinemática inversa del ciclo... ');
    x = mstraj(repmat(segments,nrep,1), [], repmat(tseg,nrep,1), ...
               segments(1,:), dt_ik, tacc);
    nspc   = round(sum(tseg)/dt_ik);
    xcycle = x(nspc+1 : 2*nspc, :);
    qcycle = leg.ikine(transl(xcycle), qHome, 'mask', [1 1 1 0 0 0]);
    qcycle(:,1) = qcycle(:,1) - mean(qcycle(:,1));
    fprintf('listo (%d muestras).\n', size(qcycle,1));

    %% ------------------ Geometría de los pies en el cuerpo ---------------
    hipXY = [  L/2 -W/2 ;  0 -R ; -L/2 -W/2 ; -L/2  W/2 ;  0  R ;  L/2  W/2 ];
    azLeg = [ -pi/4 ; -pi/2 ; -3*pi/4 ; 3*pi/4 ; pi/2 ; pi/4 ];

    pieXY = [ hipXY(:,1) + yy*cos(azLeg), hipXY(:,2) + yy*sin(azLeg) ];
    rPie  = hypot(pieXY(:,1), pieXY(:,2));
    kleg  = (rPie / mean(rPie))';       % compensación radial para el giro

    %% ------------------------ Parametros de la marcha ------------------------
    robot.gait.qcycle  = qcycle;
    robot.gait.qHome   = qHome;
    robot.gait.Tdesign = sum(tseg);     % 2.0 s
    robot.gait.stride  = stride;
    robot.gait.lift    = lift;
    robot.gait.yy      = yy;
    robot.gait.rPie    = rPie;
    robot.gait.rMedio  = mean(rPie);
    robot.gait.kleg    = kleg;
    robot.gait.fs      = 80;            % Hz de transmisión
    robot.gait.escala  = 1.0;           % 0..1
    robot.gait.useAck  = true;

    % --- Límites de velocidad -------------------------------------------

    robot.gait.Tmin    = 2.0;           % s por ciclo (más rápido permitido)
    robot.gait.Tmax    = 6.0;           % s por ciclo (más lento permitido)
    robot.gait.wmax    = 200;           % deg/s tope por junta 

    % Por ciclo hay DOS fases de apoyo.
    Dciclo = 2 * (2*stride) * robot.gait.escala;          % m por ciclo
    Gciclo = Dciclo / robot.gait.rMedio;                  % rad por ciclo

    robot.vmax = Dciclo / robot.gait.Tmin;                % m/s
    robot.vmin = Dciclo / robot.gait.Tmax;
    robot.wmax = rad2deg(Gciclo) / robot.gait.Tmin;       % deg/s
    robot.wmin = rad2deg(Gciclo) / robot.gait.Tmax;

    %% ---------------------------- Conexión -------------------------------
    fprintf('Conectando al hexápodo %d en %s:%d ...\n', ...
            robot.id, robot.ip, robot.port);

    try
        robot.tcpsock = tcpclient(robot.ip, robot.port, 'Timeout', 5);
        configureTerminator(robot.tcpsock, "LF");
    catch ME
        if strcmp(robot.red, 'robotat')
            error(['ERROR: Could not connect to the hexapod at %s\n' ...
                   '  %s\n'], ...
                   robot.ip, ME.message, robot.ip);
        else
            error(['ERROR: Could not connect to the hexapod at %s\n' ...
                   '  %s\n'], ...
                   robot.ip, ME.message);
        end
    end

    saludo = readline(robot.tcpsock);      % el ESP32 manda "READY"
    if ismissing(saludo)
        disp('Sin saludo READY del ESP32, se continúa igual.');
    else
        fprintf('ESP32: %s\n', strtrim(saludo));
    end

    fprintf('Hexápodo %d conectado (%s)\n', robot.id, robot.red);
    fprintf('  Zancada         : %.0f mm  (%.0f mm por ciclo)\n', ...
            2000*stride*robot.gait.escala, 1000*Dciclo);
    fprintf('  Velocidad       : %.1f a %.1f cm/s\n', 100*robot.vmin, 100*robot.vmax);
    fprintf('  Giro            : %.1f a %.1f deg/s  (%.1f deg por ciclo)\n', ...
            robot.wmin, robot.wmax, rad2deg(Gciclo));
    fprintf('  Altura cuerpo   : %.0f mm\n', 1000*abs(zd));

    % Pose inicial
    robotat_hexapod_force_stop(robot);
end
