in_file_name = '../cmd_robot_0';
out_file_name = '../positions_to_cgi';

w = 400;
h = 400;
x = 0;
y = 0;
v = 1;
th = 0;
kth = -0.1;

go_x = 0;  go_y = 0;

try_timeout = 10;

%figure(1)

while(1)
    f = -1;
    c = 0;
    read_ok = 1;
    while(f < 0),
        f = fopen(in_file_name, 'r');
        c = c+1;
        if(c > 1),
            fprintf('Collision avoid %d', c);
        end
        if(c >= try_timeout),
            printf('Warning:  Too many attempts to open file %s', in_file_name);
        end
    end
    
    if(read_ok),
        d = fscanf(f, '%f %f');
        fclose(f);
    
        go_x = d(1);  go_y = d(2);
    end
    
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
    
    d = round([x y]);
    
    f = -1;
    c = 0;
    write_ok = 1;
    while(f < 0),
        f = fopen(out_file_name, 'w');
        c = c+1;
        if(c > 1),
            fprintf('Collision avoid %d', c);
        end
        if(c >= try_timeout),
            printf('Warning: Too many attempts to open file %s', in_file_name);
            write_ok = 0;
        end
    end
    
    if(write_ok),
        fprintf(f, '%d %d\n', d(1), d(2));
        fclose(f);
    end
    
    %plot(x, y, 'kx', go_x, go_y, 'go')
    %axis([0 w 0 h])
            
    pause(0.1)
end