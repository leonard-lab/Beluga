function [X_o, Y_o, Z_o] = belugaSimulatorDoStep(GO_X, GO_Y, GO_Z)

% Function [X_o, Y_o, Z_o] = belugaSimulatorDoStep(GO_X, GO_Y, GO_Z)
%
% Function to simulate the closed loop dynamics of the system of 4 vehicles
%
% All inputs and outputs are 4 x 1 vectors; GO_i are waypoint coordinates
% for the four vehicles, while X_o, Y_o, Z_o are current vehicle locations.

% define persistent vehicle state variables
persistent X;       % coordinates
persistent Y;
persistent Z;
persistent TH;
persistent V1;      % velocities; V2 is assumed to be driven to 0 by low-level control
persistent V3;
persistent OMEGA;

if isempty(X),
    X = zeros(4, 1);
    Y = zeros(4, 1);
    Z = zeros(4, 1);
    TH = zeros(4, 1);
    V1 = 0.05*ones(4, 1);
    V3 = zeros(4, 1);
    OMEGA = zeros(4, 1);
end

X_o = zeros(4,1);       % initialize output variables
Y_o = zeros(4,1);
Z_o = zeros(4,1);

rmax = 3.0;     % tank size variables (m)
zmax = 2.286;

% extract the sampling rate of the simulation from the timer
t = timerfind('name','BelugaSimTimer');
dt = get(t,'Period');
clear t;

vmax = 1;       % maximum forward speed (m/s)

% control gains
kPHoriz = 0.5;  % proportional control for error -> xDot, yDot
kFbkLin = 1;    % feedback linearization gain
kz = 0.1;       % vertical controller proportional gain

for ix = 1 : 4,
    x = X(ix);
    y = Y(ix);
    z = Z(ix);
    th = TH(ix);
    v1 = V1(ix);
    v3 = V3(ix);
    omega = OMEGA(ix);
    
    go_x = GO_X(ix);
    go_y = GO_Y(ix);
    go_z = GO_Z(ix);
    
    R = [cos(th) -sin(th);  % compute the rotation matrix for future use
         sin(th)  cos(th)];
    
    % Controls
    dx = go_x - x;  % location error
    dy = go_y - y;
    dz = go_z - z;

    linVels = kPHoriz*[dx dy];  % proportional control for linear velocities
    
    bodyVels = kFbkLin*linVels*R;   % apply feedback linearization
    
    v1 = bodyVels(1); omega = bodyVels(2);   % pull out the velocities
    
    if(v1 > vmax)   % saturate forward velocity
        v1 = vmax;
    end
    
    % integrate the kinematics
    th = th + dt*omega;
    x = x + v1*cos(th);
    y = y + v1*sin(th);
    
    % apply proportional control to the z dof
    z = z + kz*dz;

    % Enforce the 'stay in the tank' constraints
    r = sqrt(x.^2 + y.^2);
    if r > rmax,
        % this is not an exact collision correction
        phi = atan2(y,x);
        rnew = rmax - (r - rmax);
        x = rnew*cos(phi);
        y = rnew*sin(phi);
        v_vec = [v1*cos(th); v1*sin(th)];
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
    V1(ix) = v1;
    V3(ix) = v3;
    OMEGA(ix) = omega;
    
    X_o(ix, 1) = x;
    Y_o(ix, 1) = y;
    Z_o(ix, 1) = z;
end