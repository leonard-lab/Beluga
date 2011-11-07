function belugaSimulator()

addpath('../matlab');

[status, r] = system('ruby ../beluga_server.rb >> simulator.log &');
if status > 0,
    error(r)
end

pause(0.1)

sock = getBelugaIPCSocket('127.0.0.1', 1234);
[go_x, go_y, go_z] = belugaGetCommandIPC(0, sock);

fig = figure();
h = plot(go_x, go_y, 'rs', 0, 0, 'go');

title('Beluga Simulator - Running')
axis([-3 3 -3 3]);

t = timer('ExecutionMode', 'fixedRate', ...
    'StartDelay', 0, ...
    'Period', 0.2);

set(t, 'TimerFcn', @(varargin) updateSimulator(fig, h, sock), ...
       'StopFcn', @(varargin) doneSimulator(fig, h, sock),...
       'name', 'BelugaSimTimer');
start(t);

function updateSimulator(f, h, sock)

[go_x_0, go_y_0, go_z_0] = belugaGetCommandIPC(0, sock);
% TODO:  [go_x_1, go_x_2, go_x_3] = belugaGetCommandIPC(1, sock);, etc

GO_X = [go_x_0];  % TODO: vectorize
GO_Y = [go_y_0];  % TODO: vectorize
GO_Z = [go_z_0];  % TODO: vectorize

[X, Y, Z] = belugaSimulatorDoStep(GO_X, GO_Y, GO_Z);

% may need to change this
set(h(1), 'XData', GO_X, 'YData', GO_Y)
set(h(2), 'XData', X, 'YData', Y)

belugaSendPositionIPC(0, X(1), Y(1), Z(1));
% TODO: belugaSendPositionIPC(1, X(2), Y(2), Z(2));

function doneSimulator(f, h, sock)

figure(f)
title('Beluga Simulator - Done');

stopBelugaServer(sock);
closeBelugaSocket(sock);