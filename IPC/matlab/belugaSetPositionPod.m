function [X, Y, Z, IX] = belugaSetPositionPod(robot_id, X, Y, Z, pod_address)

if nargin == 4,
    pod_address = 'http://127.0.0.1:3000/';
end

if(pod_address(end) ~= '/')
    pod_address = [pod_address '/'];
end

if ~isempty(find(robot_id > 3, 1))
    error('Robot id out of bounds (3 max)')
end

for ix = 1 : length(robot_id)
    url = sprintf('%spositions', pod_address);
    [resp_string, status] = urlread(url, 'post',...
        {'short', '1', 'position[id]', sprintf('%d', robot_id(ix)),...
        'position[x]', sprintf('%f', X(ix)), ...
        'position[y]', sprintf('%f', Y(ix)), ...
        'position[z]', sprintf('%f', Z(ix))});
    if(~status)
        error('Unable to determine position from pod server at url %s', url);
    end
end

[X, Y, Z, IX] = belugaParseString(resp_string, robot_id);

