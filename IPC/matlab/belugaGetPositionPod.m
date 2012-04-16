function [X, Y, Z, IX] = belugaGetPositionPod(robot_id, pod_address)

if nargin == 1,
    pod_address = 'http://127.0.0.1:3000/';
end

if(pod_address(end) ~= '/')
    pod_address = [pod_address '/'];
end

url = sprintf('%spositions?short=1', pod_address);
[resp_string, status] = urlread(url);
if(~status)
    error('Unable to determine position from pod server at url %s', url);
end

[X, Y, Z, IX] = belugaParseString(resp_string, [0 : 3]);

X = X(robot_id + 1);
Y = Y(robot_id + 1);
Z = Z(robot_id + 1);
IX = IX(robot_id + 1);
