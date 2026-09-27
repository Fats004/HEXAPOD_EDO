function info = robotat_hexapod_step(robot, sgn, kcoxa, Tciclo)
% Funciona como soporte para las funciones de giro y avance, envia los 
% pedazos de cada paso a la frecuencia deseada.

    persistent fase treloj tmuestreo

    %% ------------------------ Reinicio ------------------------------------
    if (ischar(robot) || isstring(robot)) && strcmpi(string(robot), "reset")
        fase = []; treloj = []; tmuestreo = [];
        return;
    end

    if isempty(fase),   fase = 0;            end
    if isempty(treloj), treloj = tic;        end

    G = robot.gait;

    %% -------------------- Avance de la fase -------------------------------
    dt = toc(treloj);
    treloj = tic;

    % Primera llamada, o el lazo se colgó: no dar un salto enorme.
    if dt > 0.25, dt = 1/G.fs; end

    fase = mod(fase + dt/Tciclo, 1);

    % Ritmo real de llamadas (para avisar si el lazo va muy lento)
    if isempty(tmuestreo), tmuestreo = dt; else, tmuestreo = 0.9*tmuestreo + 0.1*dt; end

    %% ------------- Interpolación del ciclo por pata ------------------------
    Q  = G.qcycle;
    M  = size(Q,1);
    ph = [0 0.5 0 0.5 0 0.5];          % trípode {1,3,5} vs {2,4,6}

    HOME  = rad2deg(G.qHome);
    qdeci = zeros(1,18);

    for i = 1:6
        fi  = mod(fase + ph(i), 1);
        idx = fi*M + 1;
        i0  = floor(idx);
        fr  = idx - i0;
        a   = mod(i0-1, M) + 1;
        b   = mod(i0,   M) + 1;
        q   = Q(a,:)*(1-fr) + Q(b,:)*fr;

        dq = [ sgn(i)*kcoxa(i)*q(1), ...
               q(2)-G.qHome(2), ...
               q(3)-G.qHome(3) ] * G.escala;

        qdeci(3*(i-1)+(1:3)) = round( (HOME + rad2deg(dq)) * 10 );
    end

    %% -------------------------- Envío -------------------------------------
    txt = sprintf('%d,', qdeci);
    writeline(robot.tcpsock, ['{"q":[' txt(1:end-1) ']}']);

    if G.useAck
        r = readline(robot.tcpsock);
        if ismissing(r)
            warning('El ESP32 no respondió a la trama.');
        end
    end

    if nargout > 0
        info.fase    = fase;
        info.dt      = dt;
        info.fs_real = 1/tmuestreo;
        info.qdeci   = qdeci;
    end
end
