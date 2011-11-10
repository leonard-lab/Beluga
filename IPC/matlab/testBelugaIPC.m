function testBelugaIPC()

try
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
    check_set_position_response(0, 1, 2, 3, sock);
    
    disp('checking set position 1')
    check_set_position_response(0, -1, -2, -3, sock);
    
    disp('checking set position 2')
    check_set_position_response(2, pi, exp(1), 0, sock);
    
    disp('checking set position 3')
    check_set_position_response(3, 1.1, 1/4, 1/pi, sock);

    disp('checking set position [1 3]')
    check_set_position_response([1 3], randn(2, 1), randn(2, 1), randn(2, 1), sock);

    disp('checking set position [0 1 2 3]')
    check_set_position_response([0 : 3], randn(4, 1), randn(4, 1), randn(4, 1), sock);    
    
    stopBelugaServer
    
catch err
    % makes sure the server stops even if a test failed
    stopBelugaServer
    rethrow(err)
end

disp 'All tests pass!'


function check_msg_response(cmd, expected, sock)

r = belugaIPCMessage(cmd, sock);

if ~strcmp(r, expected)
    error('Expected %s, got %s');
end

function check_get_position_response(id, x_ex, y_ex, z_ex, sock)

[x, y, z, id_out] = belugaGetPositionIPC(id, sock);

if ~iseqwf(x(:), x_ex(:)) || ~iseqwf(y(:), y_ex(:)) || ~iseqwf(z(:), z_ex(:)) || ~iseqwf(id(:), id_out(:))
    dump_vecs('x', x, 'x_ex', x_ex, 'y', y, 'y_ex', y_ex, 'z', z, 'z_ex', z_ex, 'id', id, 'id_ex', id_out);
    error('Unexpected response from server in check_get_position_response');
end

function check_set_position_response(id, x_ex, y_ex, z_ex, sock)

[x, y, z, id_out] = belugaSetPositionIPC(id, x_ex, y_ex, z_ex, sock);

if ~iseqwf(x(:), x_ex(:)) || ~iseqwf(y(:), y_ex(:)) || ~iseqwf(z(:), z_ex(:)) || ~iseqwf(id(:), id_out(:))
    dump_vecs('x', x, 'x_ex', x_ex, 'y', y, 'y_ex', y_ex, 'z', z, 'z_ex', z_ex, 'id', id, 'id_ex', id_out);    
    error('Unexpected response from server in check_get_position_response');
end

check_get_position_response(id, x_ex, y_ex, z_ex, sock);

function dump_vecs(varargin)

for ix = 1 : size(varargin, 2),
    disp(varargin{ix})
end

function r = iseqwf(a, b, f)

if nargin < 3,
    f = '%f';
end

r = strcmp(sprintf(f, a), sprintf(f, b));