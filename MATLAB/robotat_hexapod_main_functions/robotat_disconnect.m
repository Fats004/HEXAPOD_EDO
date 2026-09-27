function robotat_disconnect(tcp_obj)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    write(tcp_obj, uint8('EXIT'));
    disp('Disconnected from Robotat server.');
    evalin('base', ['clear ', inputname(1)]);
end