function belugaSimulator()

close all;

addpath('../matlab');

delete(timerfind)

startBelugaServer('simulator.log');

try
   
    disp('Opening socket')
    sock = getBelugaIPCSocket('127.0.0.1', 1234);
    sock.Timeout = 0.5;
    sock.Name = 'SimulatorSocket';
    sock.ErrorFcn = @(varargin) sockError();
    disp('Done')
    
    % uncomment if you want lots of output in the log file
    % belugaIPCMessage('set verbose on');
    
    TankRadius = 3.0; %roughly
    WaterDepth = 2.286; %pretty close
    
    Ts = 0.2;  % simulator sample rate
    
    % the simulator runs in waypoint mode. we initially set the waypoints to
    % evenly distributed points around the tank and just below the surface
    X_init = (2/3)*TankRadius*cos(pi/4 + pi/2*[0 : 3]);
    Y_init = (2/3)*TankRadius*sin(pi/4 + pi/2*[0 : 3]);
    Z_init = (3/4)*WaterDepth*ones(4, 1);
    [go_x, go_y, go_z] = belugaSetWaypointIPC([0 : 3], ...
        X_init, Y_init, Z_init, sock);
    
    if isempty(go_x) || isempty(go_y) || isempty(go_z)
        error('Unable to set initial waypoint')
    end 
    
    fig = figure();
    h = plot3(go_x(1), go_y(1), go_z(1), 'rs', 0, 0, 0, 'ro',...
        go_x(2), go_y(2), go_z(2), 'gs', 0, 0, 0, 'go',...
        go_x(3), go_y(3), go_z(3), 'bs', 0, 0, 0, 'bo',...
        go_x(4), go_y(4), go_z(4), 'ks', 0, 0, 0, 'ko');
    
    title('Beluga Simulator - Running')
    view(3)
    axis([-3 3 -3 3 0 3]);
    
    timer_sim = timer('ExecutionMode', 'fixedRate', ...
        'StartDelay', 0, ...
        'Period', Ts);
    
    set(timer_sim, 'TimerFcn', @(varargin) updateSimulator(fig, h, sock), ...
        'StopFcn', @(varargin) doneSimulator(fig, h, sock),...
        'name', 'BelugaSimTimer');
    start(timer_sim);

catch err
    
    % should make sure that the server gets stopped if we have an error
    if exist('sock')
        stopBelugaSimulator(sock);
    else
        stopBelugaSimulator
    end
    rethrow(err)
    
end
    
    
function updateSimulator(f, h, sock)

[GO_X, GO_Y, GO_Z, ~] = belugaGetWaypointIPC([0 : 3], sock);

if isempty(GO_X) || isempty(GO_Y) || isempty(GO_Z)
    fprintf('Warning: no response from server. skipping step.\n');
else
    
    [X, Y, Z] = belugaSimulatorDoStep(GO_X, GO_Y, GO_Z);
    
    % may need to change this
    set(h([1 : 2 : end]), {'XData'}, num2cell(GO_X), {'YData'}, num2cell(GO_Y), {'ZData'}, num2cell(GO_Z))
    set(h([2 : 2 : end]), {'XData'}, num2cell(X), {'YData'}, num2cell(Y), {'ZData'}, num2cell(Z))
    
    belugaSetPositionIPC([0 : 3], X, Y, Z, sock);
    
end 

function doneSimulator(f, h, sock)

figure(f)
title('Beluga Simulator - Done');

stopBelugaServer(sock);
closeBelugaSocket(sock);

function sockError()

puts 'A socket error has occurred in the simulator.'