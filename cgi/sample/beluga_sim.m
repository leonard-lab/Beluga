if(exist('sock', 'var')),
    fclose(sock);
end

w = 400;
h = 400;
x = 0;
y = 0;
v = 1;
th = 0;
kth = -0.1;

go_x = 0;  go_y = 0;

try_timeout = 10;

addpath('../clients')

sock = getBelugaIPCSocket('127.0.0.1', 1234);

figure(1)

while(1)
    
    [go_x, go_y, ~] = belugaGetCommandIPC(0, sock);
    
    dx = go_x - x;
    dy = go_y - y;
    
    th = th + kth*sin(th - atan2(dy,dx));
    
    x = x + v*cos(th);
    y = y + v*sin(th);
    
    if(x >= w),
        x = w - (x - w);
        th = atan2(sin(th), -cos(th));
    end
    if(x <= 0),
        x = -x;
        th = atan2(sin(th), -cos(th));
    end
    if(y >= h),
        y = y - (y - h);
        th = atan2(-sin(th), cos(th));
    end
    if(y <= 0),
        y = -y;
        th = atan2(-sin(th), cos(th));
    end
    
    belugaSendPositionIPC(0, x, y, 0, sock);
    
    plot(x, y, 'kx', go_x, go_y, 'go')
    axis([0 w 0 h])
    
    pause(0.1)
end

fclose(sock);
delete sock