function belugaGetPositionIPC(robot_id, sock)

if nargin == 1,
    sock = [];
end

resp = belugaIPCMessage(sprintf('get position %d', robot_id), sock);

resp = sscanf(resp, '%d %f %f %f');

x = resp(2);
y = resp(3);
z = resp(4);