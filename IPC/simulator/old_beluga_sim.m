if(exist('sock', 'var')),
    fclose(sock);
end

rmax = 3.2;
zmax = 2.3;
x = 0;
y = 0;
z = 0;
v = 0.05;
th = 0;
kth = -0.1;
kz = 0.1;

vmax = 0.05;
dr1 = 0.5;

go_x = 0;  go_y = 0;  go_z = 0;

try_timeout = 10;

addpath('../matlab')

sock = getBelugaIPCSocket('127.0.0.1', 1234);

figure(1)

while(1)
    
    [go_x, go_y, go_z] = belugaGetCommandIPC(0, sock);
    
    if(go_z < 0),
        go_z = 0;
    end
    if(go_z > zmax),
        go_z = zmax;
    end
    
    dx = go_x - x;
    dy = go_y - y;
    dz = go_z - z;
    
    dr = sqrt(dx^2 + dy^2);
    
    th = th + kth*sin(th - atan2(dy,dx));
    
    if(dr > dr1)
        v = vmax;
    else
        v = vmax*(dr/dr1);
    end
    
    x = x + v*cos(th);
    y = y + v*sin(th);
    
    z = z + kz*dz;
    
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
    
    if(z < 0)
        z = 0;
    end
    if(z > zmax)
        z = zmax;
    end
    
    belugaSendPositionIPC(0, x, y, z, sock);
    
    plot(x, y, 'kx', go_x, go_y, 'go')
    axis(1.1*[-rmax rmax -rmax rmax])
    
    pause(0.1)
end

fclose(sock);
delete sock