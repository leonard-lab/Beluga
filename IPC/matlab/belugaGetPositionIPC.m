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

resp = sscanf(resp_string, '%f ');

if length(resp) < 3*num_bots,
    fprintf('Unexpected response from server in %s: %s\n', mfilename, resp_string);
    X = -1*ones(num_bots, 1);
    Y = -1*ones(num_bots, 1);
    Z = -1*ones(num_bots, 1);
    IX = -1*ones(num_bots, 1);
else
    for ix = 1 : num_bots,
        X(ix) = resp(3*(ix-1) + 1);
        Y(ix) = resp(3*(ix-1) + 2);
        Z(ix) = resp(3*(ix-1) + 3);
        IX(ix) = robot_id(ix);
    end
end

