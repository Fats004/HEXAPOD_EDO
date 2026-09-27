function info = robotat_hexapod_stream(robot, sgn, kcoxa, Tciclo, duracion, etiqueta)

%   El ciclo se REMUESTREA a exactamente round(Tciclo*fs), para
%   que reproducirlo a fs Hz dure exactamente Tciclo. 

    G  = robot.gait;
    fs = G.fs;
    Ts = 1/fs;

    %% ------------------- Remuestreo del ciclo ----------------------------
    N = max(8, round(Tciclo * fs));
    M = size(G.qcycle, 1);

    tOrig = [(0:M-1)'/M ; 1];                  % periodo normalizado, cerrado
    qOrig = [G.qcycle ; G.qcycle(1,:)];
    tNew  = (0:N-1)'/N;
    qres  = interp1(tOrig, qOrig, tNew, 'linear');

    %% ---------------- Armado de la tabla de 18 ángulos -------------------
    off   = round(N/2);
    phase = [0 off 0 off 0 off];               % trípode {1,3,5} vs {2,4,6}
    HOME  = rad2deg(G.qHome);

    QDECI = zeros(N, 18);
    for k = 1:N
        for i = 1:6
            kk = mod(k-1+phase(i), N) + 1;
            q  = qres(kk,:);
            dq = [ sgn(i)*kcoxa(i)*q(1), ...
                   q(2)-G.qHome(2), ...
                   q(3)-G.qHome(3) ] * G.escala;
            QDECI(k, 3*(i-1)+(1:3)) = round( (HOME + rad2deg(dq)) * 10 );
        end
    end

    %% ------------- Verificación de velocidad articular -------------------
    dQ    = abs(diff([QDECI ; QDECI(1,:)], 1, 1)) / 10;   % deg entre tramas
    wpico = max(dQ(:)) / Ts;                              % deg/s

    if wpico > G.wmax
        warning(['Velocidad articular pico %.0f deg/s (tope recomendado %.0f). ' ...
                 'Los AX-12A pueden no seguir la consigna. Bajá la velocidad.'], ...
                wpico, G.wmax);
    end

    info.N      = N;
    info.Tciclo = Tciclo;
    info.wpico  = wpico;
    info.ciclos = duracion / Tciclo;

    fprintf('%s | %d tramas/ciclo | T = %.2f s | pico articular %.0f deg/s\n', ...
            etiqueta, N, Tciclo, wpico);

    %% -------------------------- Streaming ------------------------------
    limpieza = onCleanup(@() robotat_hexapod_force_stop(robot)); 
    k = 1;  tramas = 0;  tsig = 0;
    treloj = tic;

    while toc(treloj) < duracion
        txt = sprintf('%d,', QDECI(k,:));
        writeline(robot.tcpsock, ['{"q":[' txt(1:end-1) ']}']);

        if G.useAck
            r = readline(robot.tcpsock);
            if ismissing(r)
                warning('El ESP32 dejó de responder. Se aborta la marcha.');
                break;
            end
        end

        tramas = tramas + 1;
        k = mod(k, N) + 1;

        tsig = tsig + Ts;
        espera = tsig - toc(treloj);
        if espera > 0, pause(espera); end
    end

    t_real = toc(treloj);
    info.tramas   = tramas;
    info.t_real   = t_real;
    info.fs_real  = tramas / t_real;
    info.ciclos   = tramas / N;

    fprintf('  %d tramas en %.2f s (%.1f Hz reales, %.2f ciclos)\n', ...
            tramas, t_real, info.fs_real, info.ciclos);
end
