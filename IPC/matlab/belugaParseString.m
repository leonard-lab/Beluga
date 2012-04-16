function [A, B, C, IX] = belugaParseString(resp_string, robot_id)

num_bots = numel(robot_id);
if ~isempty(find(robot_id > 3, 1))
    error('Robot id out of bounds (3 max)')
end

resp = sscanf(resp_string, '%f ');

A = zeros(num_bots, 1);
B = zeros(num_bots, 1);
C = zeros(num_bots, 1);
IX = zeros(num_bots, 1);

if length(resp) ~= 3*num_bots,
    fprintf('Unexpected response from server in %s: %s\n', mfilename, resp_string);
    A = -1*ones(num_bots, 1);
    B = -1*ones(num_bots, 1);
    C = -1*ones(num_bots, 1);
    IX = -1*ones(num_bots, 1);
else
    for ix = 1 : num_bots,
        A(ix) = resp(3*(ix-1) + 1);
        B(ix) = resp(3*(ix-1) + 2);
        C(ix) = resp(3*(ix-1) + 3);
        IX(ix) = robot_id(ix);
    end
end