function robotat_hexapod_disconnect(robot)

    robotat_hexapod_force_stop(robot);
    pause(1.5);                      % darle tiempo de llegar a HOME

    evalin('base', ['clear ', inputname(1)]);
    disp('Disconnected from hexapod.');
end
