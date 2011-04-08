function belugaSendPositionIPC(robot_id, x, y, z, sock)

if nargin == 4,
    sock = [];
end

belugaIPCMessage(sprintf('set position %d %f %f %f', robot_id, x, y, z), sock);