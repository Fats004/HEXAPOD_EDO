function tcp_obj = robotat_connect()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    ip = '192.168.50.200';
    port = 1883;
    try
        tcp_obj = tcpclient(ip, port);
    catch
        disp('ERROR: Could not connect to Robotat server.');
    end
end