clear all;

addpath('../matlab/');
addpath('../simulator');

if ~isBelugaServerRunning,
    error('The Beluga IPC server is not running.')
end

sock = getBelugaIPCSocket('127.0.0.1', 1234);
if ~strcmp(sock.status, 'open')
    error('Unable to connect to Beluga IPC server.')
end

t0 = tic;
t = toc(t0);

N = 2;

R = 1.0;
Rtank = 3;
omega = 2*pi/60;  % 60 seconds around the loop
C1 = -0.5*Rtank + R*exp(i*linspace(0, 2*pi));
C2 = 0.5*Rtank + R*exp(i*linspace(0, 2*pi));
C = [C1; C2];

T_stop = 1*2*pi/omega;

% sending this makes the waypoint controller ignore the z command
NO_Z = -1;

[X_now, Y_now, ~] = belugaGetPositionIPC([0 : N-1], sock);
X0 = [-0.5*Rtank + R; 0.5*Rtank + R];
Y0 = [0; 0];

figure(2);
h = plot(X_now, Y_now, 'o', X0, Y0, 's', real(C)', imag(C)', 'k-');
axis equal

while(t < T_stop)
    t = toc(t0);
    tstart = t;
    
    X = [-0.5*Rtank; 0.5*Rtank] + R*cos(omega*t);
    Y = [0; 0] + R*sin(omega*t);
    
    belugaSetWaypointIPC([0 : N-1], X, Y, NO_Z*[1; 1], sock);
    
    [X_now, Y_now] = belugaGetPositionIPC([0 1], sock);
    set(h(1), 'XData', X_now, 'YData', Y_now);
    set(h(2), 'XData', X, 'YData', Y);
    
    t = toc(t0);
    pause(0.25 - (t - tstart));
end

closeBelugaSocket(sock);