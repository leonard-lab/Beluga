function [X, Y, Z, SPD, OMEGA, ZDOT, IX] = belugaSetControlIPC(robot_id, ...
    x, y, z, spd, omega, zdot, sock)

if nargin == 1,
    sock = [];
end

msg = 'set control ';
mode_in = '';
if isempty(x) && isempty(y) && isempty(z) && ...
        ~isempty(spd) && ~isempty(omega) && ~isempty(zdot),
    mode_in = 'kinematics';
    d1 = spd;  d2 = omega;  d3 = zdot;
elseif ~isempty(x) && ~isempty(y) && ~isempty(z) && ...
        isempty(spd) && isempty(omega) && isempty(zdot),
    mode_in = 'waypoint';
    d1 = x; d2 = y; d3 = z;
else
    error(['Unable to determine control mode.  Either set (x,y,z) or ' ...
        '(speed, omega, zdot) and leave the others empty.']);
end

msg = [msg mode_in ' '];

num_bots = length(robot_id);
for ix = 1 : num_bots,
    r_id = robot_id(ix);
    if(r_id > 3)
        error('Robot id %d is out of bounds (3 max)', r_id)
    end
    msg = [msg int2str(r_id) sprintf(' %f %f %f ', d1(ix), d2(ix), d3(ix))];
end

resp_string_orig = belugaIPCMessage(msg, sock);
resp_string = resp_string_orig;

mode_pos = 1;
mode = '';
while ~isempty(resp_string) && mode_pos == 1,
    mode_pos = strfind(resp_string, ' ');
    if isempty(mode_pos)
        mode_pos = length(resp_string)-1;
    else
        mode_pos = mode_pos(1);
    end
    if mode_pos == 1,
        resp_string = resp_string([2 : end]);
    else
        mode = resp_string([1 : mode_pos-1]);
        if(mode_pos+1 == length(resp_string))
            resp_string = '';
        else
            resp_string = resp_string([mode_pos+1 : end]);
        end
    end
end

X = [];
Y = [];
Z = [];
SPD = [];
OMEGA = [];
ZDOT = [];
IX = zeros(num_bots, 1);

resp = sscanf(resp_string, '%f ');

if length(resp) < 3*num_bots,
    fprintf('Unexpected response from server in %s: %s\n', mfilename, resp_string);
    X = [];
    Y = [];
    Z = [];
    SPD = [];
    OMEGA = [];
    ZDOT = [];    
    IX = -1*ones(num_bots, 1);
else
    if strcmp(mode, 'waypoint'),
        X = resp(1 : 3 : end);
        Y = resp(2 : 3 : end);
        Z = resp(3 : 3 : end);
        IX = robot_id;
    elseif strcmp(mode, 'kinematics'),
        SPD = resp(1 : 3 : end);
        OMEGA = resp(2 : 3 : end);
        ZDOT = resp(3 : 3 : end);
        IX = robot_id;
    else
        fprintf('Unexpected mode recieved from serverin %s: %s\n', mfilename, mode);
    end
end

