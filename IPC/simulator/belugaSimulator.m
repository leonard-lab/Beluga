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

[go_x, go_y, go_z] = belugaGetCommandIPC(0, sock);
[x, y, z] = belugaSimulatorDoStep(go_x, go_y, go_z);

set(h(1), 'XData', go_x, 'YData', go_y)
set(h(2), 'XData', x, 'YData', y)

belugaSendPositionIPC(0, x, y, z);

function doneSimulator(f, h, sock)

figure(f)
title('Beluga Simulator - Done');

stopBelugaServer(sock);
closeBelugaSocket(sock);