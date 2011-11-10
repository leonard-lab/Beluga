function [X_o, Y_o, Z_o] = belugaSimulatorDoStep(GO_X, GO_Y, GO_Z)

persistent X;
persistent Y;
persistent Z;
persistent TH;
persistent V;

if isempty(X),
    X = zeros(4,1);
    Y = zeros(4,1);
    Z = zeros(4,1);
    TH = zeros(4,1);
    V = 0.05*ones(4,1);
end

rmax = 3.2;
zmax = 2.3;

kth = -0.1;
kz = 0.1;

vmax = 0.05;
dr1 = 0.5;

for ix = 1 : 4,
    x = X(ix);
    y = Y(ix);
    z = Z(ix);
    th = TH(ix);
    v = V(ix);
    
    go_x = GO_X(ix);
    go_y = GO_Y(ix);
    go_z = GO_Z(ix);
    
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
    
    X(ix) = x;
    Y(ix) = y;
    Z(ix) = z;
    TH(ix) = th;
    V(ix) = v;
    
    X_o(ix) = x;
    Y_o(ix) = y;
    Z_o(ix) = z;
end