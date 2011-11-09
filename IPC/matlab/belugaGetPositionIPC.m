function [x, y, z] = belugaGetPositionIPC(robot_id, sock)

if nargin == 1,
    sock = [];
end

resp_string = belugaIPCMessage(sprintf('get position %d', robot_id), sock);

resp = sscanf(resp_string, '%d %f %f %f');

if(length(resp) < 3)
    fprintf('Unexpected response from server: %s', resp_string)
    x = -1;
    y = -1;
    z = -1;
else
    x = resp(2);
    y = resp(3);
    z = resp(4);
end