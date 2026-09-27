function robotat_trvisualize(tcp_obj, agents_ids)
    
    Txyz = @(a, b, c) [1,0,0,a; 0,1,0,b; 0,0,1,c; 0,0,0,1];
    Trx = @(t) [1,0,0,0; 0,cosd(t),-sind(t),0; 0,sind(t),cosd(t),0; 0,0,0,1];
    Try = @(t) [cosd(t),0,sind(t),0; 0,1,0,0; -sind(t),0,cosd(t),0; 0,0,0,1];
    Trz = @(t) [cosd(t),-sind(t),0,0; sind(t),cosd(t),0,0; 0,0,1,0; 0,0,0,1];

    mocap_data = robotat_get_pose(tcp_obj, agents_ids, 'eulzyx');
    N = numel(agents_ids);
    xframe_handles = zeros(1, N);
    yframe_handles = zeros(1, N);
    zframe_handles = zeros(1, N);
    name_handles = zeros(1, N);

    figure('units', 'normalized', 'outerposition', [0 0.05 1 0.95]);
    hold on;
    view([37.5, 30]);
    
    button_handle = uicontrol('Style', 'PushButton', 'String', 'Close', ...
                         'Callback', 'delete(gcbf)');

    for i = 1:N
        xi = mocap_data(i, :);
        T = Txyz(xi(1), xi(2), xi(3)) * Trz(xi(4)) * Try(xi(5)) * Trx(xi(6));
        o = T(1:3, 4);
        R = T(1:3, 1:3);
        x = 0.25*R(1:end, 1); 
        y = 0.25*R(1:end, 2);
        z = 0.25*R(1:end, 3);

        xframe_handles(i) = plot3([o(1), o(1)+x(1)], [o(2), o(2)+x(2)], ...
            [o(3), o(3)+x(3)], 'LineWidth', 2, 'Color', [0.6350, 0.0780, 0.1840]);
        yframe_handles(i) = plot3([o(1), o(1)+y(1)], [o(2), o(2)+y(2)], ...
            [o(3), o(3)+y(3)], 'LineWidth', 2, 'Color', [0, 0.4470, 0.7410]);
        zframe_handles(i) = plot3([o(1), o(1)+z(1)], [o(2), o(2)+z(2)], ...
            [o(3), o(3)+z(3)], 'LineWidth', 2, 'Color', [0.4660, 0.6740, 0.1880]);
        name_handles(i) = text(o(1), o(2), o(3), ...
            ['\{', num2str(agents_ids(i)),'\}']);
    end
    
    grid on;
    axis equal;
    xlim([-3, 3]);
    ylim([-3, 3]);
    zlim([-0.1, 2]);
    xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 18);
    ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 18);
    zlabel('$z$', 'Interpreter', 'latex', 'FontSize', 18);
    hold off;

    while(true)
        try
            mocap_data = robotat_get_pose(tcp_obj, agents_ids, 'eulzyx');
        catch
        end
        
        if(~ishandle(button_handle))
            break;
        end
        
        for i = 1:N
            xi = mocap_data(i, :);
            T = Txyz(xi(1), xi(2), xi(3)) * Trz(xi(4)) * Try(xi(5)) * Trx(xi(6));
            o = T(1:3, 4);
            R = T(1:3, 1:3);
            x = 0.25*R(1:end, 1); 
            y = 0.25*R(1:end, 2);
            z = 0.25*R(1:end, 3);

            set(xframe_handles(i), 'XData', [o(1), o(1)+x(1)]);
            set(xframe_handles(i), 'YData', [o(2), o(2)+x(2)]);
            set(xframe_handles(i), 'ZData' , [o(3), o(3)+x(3)]);

            set(yframe_handles(i), 'XData', [o(1), o(1)+y(1)]);
            set(yframe_handles(i), 'YData', [o(2), o(2)+y(2)]);
            set(yframe_handles(i), 'ZData', [o(3), o(3)+y(3)]);

            set(zframe_handles(i), 'XData', [o(1), o(1)+z(1)]);
            set(zframe_handles(i), 'YData', [o(2), o(2)+z(2)]);
            set(zframe_handles(i), 'ZData', [o(3), o(3)+z(3)]);

            set(name_handles(i), 'Position', o');
        end

    end
end