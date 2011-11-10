function [X, Y, Z, IX] = belugaGetPositionIPC(robot_id, sock)

if nargin == 1,
    sock = [];
end

msg = 'get position ';
num_bots = length(robot_id);
for ix = 1 : num_bots,
    r_id = robot_id(ix);
    if(r_id > 3)
        error('Robot id %d is out of bounds (3 max)', r_id)
    end
    msg = [msg int2str(r_id) ' '];
end

resp_string = belugaIPCMessage(msg, sock);

X = zeros(num_bots, 1);
Y = zeros(num_bots, 1);
Z = zeros(num_bots, 1);
IX = zeros(num_bots, 1);

for ix = 1 : num_bots,
    resp = sscanf(resp_string, '%f %f %f');
    
    if(length(resp) < 3)
        fprintf('Unexpected response from server: %s', resp_string)
        X(ix) = -1;
        Y(ix) = -1;
        Z(ix) = -1;
        IX(ix) = -1;
    else
        X(ix) = resp(1);
        Y(ix) = resp(2);
        Z(ix) = resp(3);
        IX(ix) = robot_id(ix);
    end
end

