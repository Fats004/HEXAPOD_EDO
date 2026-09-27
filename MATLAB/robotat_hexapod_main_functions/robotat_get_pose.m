function mocap_data = robotat_get_pose(tcp_obj, agents_ids, rotrep)
    if(nargin == 2)
        rotrep = 'quat';
    end

    timeout_count = 0;
    timeout_in100ms = 1 / 0.1;
    read(tcp_obj);

    if((min(agents_ids) > 0) && (max(agents_ids) <= 100))
        s.dst = 1; % DST_ROBOTAT
        s.cmd = 1; % CMD_GET_POSE
        s.pld = round(agents_ids);
        write(tcp_obj, uint8(jsonencode(s)));

        while((tcp_obj.BytesAvailable == 0) && (timeout_count < timeout_in100ms))
            timeout_count = timeout_count + 1;
            pause(0.1);
        end

        if(timeout_count == timeout_in100ms)
            disp('ERROR: Could not receive data from server.');
            return;
        else
            
            mocap_data = jsondecode(char(read(tcp_obj)));
            mocap_data = reshape(mocap_data, [7, numel(agents_ids)])';

            switch lower(rotrep)
                case 'quat'
                    % pass

                case 'eulzyz'
                    for i = 1:size(mocap_data, 1)
                        mocap_data(i, 4:end-1) = rad2deg( q2eul(mocap_data(i, 4:end), 'ZYZ') );
                    end
                    mocap_data(:, end) = [];
                
                case 'eulzyx'
                    for i = 1:size(mocap_data, 1)
                        mocap_data(i, 4:end-1) = rad2deg( q2eul(mocap_data(i, 4:end), 'ZYX') );
                    end
                    mocap_data(:, end) = [];

                case 'eulxyz'
                    for i = 1:size(mocap_data, 1)
                        mocap_data(i, 4:end-1) = rad2deg( q2eul(mocap_data(i, 4:end), 'XYZ') );
                    end
                    mocap_data(:, end) = [];

                otherwise
                    error('Invalid rotation representation.');

            end

            % if(strcmp(rotrep, 'quat'))
            %     % pass
            % else
            %     try
            %         mocap_data(:, 4:end-1) = rad2deg(quat2eul(mocap_data(:, 4:end), upper(rotrep(end-2:end)) ));
            %         mocap_data(:, end) = [];
            %     catch
            %         error('Invalid Euler angle sequence.');
            %     end
            % end

        end
    else
        disp('ERROR: Invalid ID(s).');
    end
end