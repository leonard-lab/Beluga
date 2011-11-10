function testBelugaIPC()

startBelugaServer('test.log');

sock = getBelugaIPCSocket('127.0.0.1', 1234);

disp('checking ping response')
check_msg_response('ping', 'PONG!', sock);

disp('checking get position 0')
check_get_position_response(0, 0, 0, 0, sock);

disp('checking get position 1')
check_get_position_response(1, 0, 0, 0, sock);

disp('checking get position 2')
check_get_position_response(2, 0, 0, 0, sock);

disp('checking get position 3')
check_get_position_response(3, 0, 0, 0, sock);

disp('checking get position [0 2]')
check_get_position_response([0 2], [0 0], [0 0], [0 0], sock);

disp('checking get position [0 1 2 3]')
check_get_position_response([0 1 2 3], [0 0 0 0], [0 0 0 0], [0 0 0 0], sock);

disp('checking set position 0')
check_set_position_response(0, 0, 0, 0, sock);

stopBelugaServer

disp 'All tests pass!'


function check_msg_response(cmd, expected, sock)

r = belugaIPCMessage(cmd, sock);

if ~strcmp(r, expected)
    error('Expected %s, got %s');
end

function check_get_position_response(id, x_ex, y_ex, z_ex, sock)

[x, y, z, id_out] = belugaGetPositionIPC(id, sock);

if ~isequal(x(:), x_ex(:)) || ~isequal(y(:), y_ex(:)) || ~isequal(z(:), z_ex(:)) || ~isequal(id(:), id_out(:))
    error('Unexpected response from server in check_get_position_response');
end

function check_set_position_response(id, x_ex, y_ex, z_ex, sock)

[x, y, z, id_out] = belugaSetPositionIPC(id, x_ex, y_ex, z_ex, sock);

if ~isequal(x(:), x_ex(:)) || ~isequal(y(:), y_ex(:)) || ~isequal(z(:), z_ex(:)) || ~isequal(id(:), id_out(:))
    error('Unexpected response from server in check_get_position_response');
end

check_get_position_response(id, x_ex, y_ex, z_ex, sock);