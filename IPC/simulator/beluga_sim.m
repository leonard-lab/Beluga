if(exist('sock', 'var')),
    fclose(sock);
end

rmax = 3.2;
x = 0;
y = 0;
v = 0.05;
th = 0;
kth = -0.1;

go_x = 0;  go_y = 0;

try_timeout = 10;

addpath('../matlab')

sock = getBelugaIPCSocket('127.0.0.1', 1234);

figure(1)

while(1)
    
    [go_x, go_y, ~] = belugaGetCommandIPC(0, sock);
    
    dx = go_x - x;
    dy = go_y - y;
    
    th = th + kth*sin(th - atan2(dy,dx));
    
    x = x + v*cos(th);
    y = y + v*sin(th);
    
    r = sqrt(x.^2 + y.^2);
    if r > rmax,
        % this is not an exact collision correction
        phi = atan2(y,x);
        rnew = rmax - (r - rmax);
        x = rnew*cos(phi);
        y = rnew*sin(phi);
        v_vec = [v*cos(th); v*sin(th)];
        v_radial = [cos(phi) sin(phi)]*v_vec;
        v_perp = [-sin(phi) cos(phi)]*v_vec;
        v_vec_new = -v_radial*[cos(phi); sin(phi)] + v_perp*[-sin(phi); cos(phi)];
        th = atan2(v_vec_new(2), v_vec_new(1));
    end
    
    belugaSendPositionIPC(0, x, y, 0, sock);
    
    plot(x, y, 'kx', go_x, go_y, 'go')
    axis(1.1*[-rmax rmax -rmax rmax])
    
    pause(0.1)
end

fclose(sock);
delete sock