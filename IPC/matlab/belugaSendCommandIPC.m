function belugaSendCommandIPC(robot_id, go_x, go_y, go_z, sock)

if(nargin == 4)
    sock = [];
end

belugaIPCMessage(sprintf('set command %d %f %f %f', robot_id, go_x, go_y, go_z), sock);