function robotat_hexapod_force_stop(robot)

    % Reiniciar la fase incluso si el socket está muerto
    robotat_hexapod_step('reset');

    if ~isfield(robot, 'tcpsock') || ~isvalid(robot.tcpsock)
        warning('Socket no válido: no se pudo enviar HOME.');
        return;
    end

    try
        writeline(robot.tcpsock, '{"c":"home"}');
        if ~isfield(robot,'gait') || robot.gait.useAck
            r = readline(robot.tcpsock);
            if ismissing(r) || ~contains(r, "ok")
                warning('El ESP32 no confirmó el HOME.');
            end
        end
    catch ME
        warning('Fallo al enviar HOME: %s', ME.message);
    end
end
