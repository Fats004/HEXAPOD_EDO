function info = robotat_hexapod_turn_gait(robot, direccion, velocidad)
%
%   velocidad : deg/s. Se satura a [robot.wmin, robot.wmax].
%
%   robot.wmin = 8
%   robot.wmin = 24
%
%   --------------------------- GIRO --------------------------------
%   El ciclo de cada pata el mismo que en el de avance. Lo único
%   que cambia es el vector sgn:
%
%       traslación -> sgn = [+1 +1 +1 -1 -1 -1]   fuerza neta -> avanza
%       giro       -> sgn = [+1 +1 +1 +1 +1 +1]   par neto    -> gira
%
%   Los pies no están todos al mismo radio del centro (esquinas 250 mm,  
%   medias 211 mm). Una pata a menor radio necesita menos barrido. 
%
%       G_ciclo = 2*(2*stride)/r_medio         omega = G_ciclo / T_ciclo

    %% -------------------------- Dirección --------------------------------
    if ischar(direccion) || isstring(direccion)
        switch lower(string(direccion))
            case {"left","izquierda","izq","l"},  d = +1;
            case {"right","derecha","der","r"},   d = -1;
            otherwise
                error('Dirección inválida. Usá ''left'' o ''right''.');
        end
    else
        d = sign(direccion);
        if d == 0, error('Dirección inválida.'); end
    end

    %% ------------------------- Velocidad ---------------------------------
    if velocidad < 0
        error('La velocidad debe ser positiva.');
    end

    if velocidad > robot.wmax
        warning(['Angular speed saturated to ', num2str(robot.wmax,'%.1f'), ...
                 ' deg/s (límite seguro de los AX-12A).']);
        velocidad = robot.wmax;
    elseif velocidad < robot.wmin
        warning(['Angular speed saturated to ', num2str(robot.wmin,'%.1f'), ...
                 ' deg/s (por debajo el ciclo es demasiado lento).']);
        velocidad = robot.wmin;
    end

    Dciclo = 2 * (2*robot.gait.stride) * robot.gait.escala;
    Gciclo = rad2deg( Dciclo / robot.gait.rMedio );      % deg por ciclo
    Tciclo = Gciclo / velocidad;

    %% ------------------------- Coordinación ------------------------------
    sgn   = d * [ +1 +1 +1 +1 +1 +1 ];
    kcoxa = robot.gait.kleg;

    if nargout > 0
        info = robotat_hexapod_step(robot, sgn, kcoxa, Tciclo);
        info.velocidad = d * velocidad;
        info.Tciclo    = Tciclo;
        info.Gciclo    = Gciclo;
    else
        robotat_hexapod_step(robot, sgn, kcoxa, Tciclo);
    end
end
