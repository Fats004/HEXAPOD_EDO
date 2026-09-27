function info = robotat_hexapod_advance_gait(robot, direccion, velocidad)
%   velocidad medida en m/s.
%
%   robot.vmax = 0.10
%   robot.vmin = 0.03
%
%   -------------------------- MARCHA TRÍPODE -----------------------------
%   Cada pata recorre un rectángulo en su marco local, empuja hacia atrás
%   apoyado (2*stride = 100mm approx.), se levanta, regresa adelante por 
%   el aire y baja. 
%   
%   Las extremidades {1,3,5} y {2,4,6} van en intercaladas, 
%   así que SIEMPRE hay tres patas en el suelo de apoyo. 
% 
%   Como hay DOS fases de apoyo por ciclo, el cuerpo avanza el doble de 
%   la zancada:
%
%       D_ciclo = 2*(2*stride) = 200mm approx.       v = D_ciclo / T_ciclo
%
%   El vector sgn invierte el barrido de la coxa en las patas 4-5-6 porque
%   están montadas al otro lado.

    %% -------------------------- Dirección -------------------------------
    if ischar(direccion) || isstring(direccion)
        switch lower(string(direccion))
            case {"forward","adelante","fwd","f"},         d = +1;
            case {"backward","atras","atrás","back","b"},  d = -1;
            otherwise
                error('Dirección inválida. Usá ''forward'' o ''backward''.');
        end
    else
        d = sign(direccion);
        if d == 0, error('Dirección inválida.'); end
    end

    %% ------------------------- Velocidad --------------------------------
    if velocidad < 0
        error('La velocidad debe ser positiva.');
    end

    if velocidad > robot.vmax
        warning(['Speed saturated to ', num2str(robot.vmax,'%.3f'), ' m/s ', ...
                 '(límite seguro de los AX-12A).']);
        velocidad = robot.vmax;
    elseif velocidad < robot.vmin
        warning(['Speed saturated to ', num2str(robot.vmin,'%.3f'), ' m/s ', ...
                 '(por debajo el ciclo es demasiado lento).']);
        velocidad = robot.vmin;
    end

    Dciclo = 2 * (2*robot.gait.stride) * robot.gait.escala;
    Tciclo = Dciclo / velocidad;

    %% ------------------------- Coordinación -----------------------------
    sgn   = d * [ +1 +1 +1 -1 -1 -1 ];
    kcoxa = ones(1,6);                 

    if nargout > 0
        info = robotat_hexapod_step(robot, sgn, kcoxa, Tciclo);
        info.velocidad = d * velocidad;
        info.Tciclo    = Tciclo;
    else
        robotat_hexapod_step(robot, sgn, kcoxa, Tciclo);
    end
end
