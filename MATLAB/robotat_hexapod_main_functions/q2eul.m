function eul = q2eul(q, seq)
    
    % Unit cuaternion check, comment for speed
    unity_norm_tol = 0.1;
    if((numel(q) ~= 4) || (norm(q) - 1 > unity_norm_tol))
        error('Input is NOT a unit quaternion');
    end

    if(nargin == 1)
        seq = 'ZYX';
    end
    
    % Conversión de cuaternión a ángulos de Euler
    % https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9648712/
    % TODO: implementar el algoritmo completo para considerar todas las
    % secuencias
    angcomp_tol = 0.1 * (pi/180);
    
    switch lower(seq)
        case 'zyz'
            eul = fliplr([ atan2(q(4), q(1)) - atan2(-q(2), q(3)), ...
                acos(2*(q(1)^2 + q(4)^2) - 1), ...
                atan2(q(4), q(1)) + atan2(-q(2), q(3)) ]);

            if(abs(eul(2) - 0) < angcomp_tol)
                warning('Euler angle sequence is in a singularity.');
                eul(3) = eul(3) + eul(1);
                eul(1) = 0;
            elseif(abs(abs(eul(2)) - pi) < angcomp_tol)
                warning('Euler angle sequence is in a singularity.');
                eul(3) = eul(3) - eul(1);
                eul(1) = 0;
            end


        case 'zyx'
            eul = fliplr([ atan2(q(2)+q(4), q(1)-q(3)) - atan2(q(4)-q(2), q(3)+q(1)), ...
                    acos((q(1)-q(3))^2 + (q(2)+q(4))^2 - 1) - pi/2, ...
                    atan2(q(2)+q(4), q(1)-q(3)) + atan2(q(4)-q(2), q(3)+q(1)) ]);

            if(abs(abs(eul(2)) - (pi/2)) < angcomp_tol)
                warning('Euler angle sequence is in a singularity.');
                eul(1) = eul(1) - sign(eul(2))*eul(3);
                eul(3) = 0;
            end


        case 'xyz'
            eul = fliplr([ atan2(q(4)-q(2), q(1)-q(3)) + atan2(q(2)+q(4), q(3)+q(1)), ...
                acos((q(1)-q(3))^2 + (q(4)-q(2))^2 - 1) - pi/2, ...
                -(atan2(q(4)-q(2), q(1)-q(3)) - atan2(q(2)+q(4), q(3)+q(1)))]);

            if(abs(abs(eul(2)) - (pi/2)) < angcomp_tol)
                warning('Euler angle sequence is in a singularity.');
                eul(3) = eul(3) + sign(eul(2))*eul(1);
                eul(1) = 0;
            end

        otherwise
            error('Invalid Euler angle sequence.');

    end

end