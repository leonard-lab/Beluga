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

R = 2.0;
omega = 2*pi/60;  % 60 seconds around the loop
C = R*exp(i*linspace(0, 2*pi));
T_stop = 1*2*pi/omega;

% sending this makes the waypoint controller ignore the z command
NO_Z = 1;

[x_now, y_now] = belugaGetPositionIPC(0, sock);
x0 = R;
y0 = 0;

figure(2);
h = plot(x_now, y_now, 'go', x0, y0, 'gs', real(C), imag(C), 'k-');
axis equal

while(t < T_stop)
    t = toc(t0);
    tstart = t;
    
    x = R*cos(omega*t);
    y = R*sin(omega*t);
    
    belugaSetWaypointIPC(0, x, y, NO_Z, sock);
    
    [x_now, y_now] = belugaGetPositionIPC(0, sock);
    set(h(1), 'XData', x_now, 'YData', y_now);
    set(h(2), 'XData', x, 'YData', y);
    
    t = toc(t0);
    pause(0.25 - (t - tstart));
end

closeBelugaSocket(sock);