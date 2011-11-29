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
persistent ZDOT;
persistent TH;
persistent V1;      % velocities; V2 is assumed to be driven to 0 by low-level control
persistent V3;
persistent OMEGA;

persistent H;
persistent T_ALL;
persistent X_ALL;
persistent Y_ALL;
persistent Z_ALL;
persistent GX_ALL;
persistent GY_ALL;
persistent GZ_ALL;
persistent ZDOT_ALL;
persistent TH_ALL;
persistent V1_ALL;
persistent OMEGA_ALL;
persistent UT_ALL;
persistent US_ALL;
persistent UV_ALL;
persistent uT_ALL;
persistent uS_ALL;
persistent uV_ALL;

if isempty(X),
    X = zeros(4, 1);
    Y = zeros(4, 1);
    Z = zeros(4, 1);
    ZDOT = zeros(4,1);
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
dt = get(t,'InstantPeriod');
if(isnan(dt)),
    dt = get(t, 'Period');
end

if(isempty(T_ALL)),
    t_now = 0;
else
    t_now = T_ALL(end) + dt;
end

clear t;

vmax = 1;       % maximum forward speed (m/s)

% control gains CAN STILL USE TUNING
kPHoriz = 0.5;    % proportional control for error -> xDot, yDot
kFbkLin = 0.5;    % feedback linearization gain
kz = 0.5;       % vertical controller proportional gain

% dynamics constants
n_up = 0.0096;       % motor up efficiency
n_down = 0.1080;     % motor down efficiency
v_offset = 0.2;      % vertical offset velocity [m/sec]
k_vert = 11.25;
K_d = 70;
K_t = 1.1;
z_offset = 0.8;
m_total = 14.4;
K_T3 = 1.03;
K_d1 = 45;
r_torque = 0.35;
K_omega = 7;
J = 2.5;

U = zeros(4, 3);
u = zeros(4, 3);

for ix = 1 : 4,
    x = X(ix);
    y = Y(ix);
    z = Z(ix);
    zdot = ZDOT(ix);
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
    
    u_speed = bodyVels(1); % the speed and omega that we want
    u_omega = bodyVels(2);   
    
    % kinematic controls - based on steady-state speed/omega
    u_thrust = (K_d1/K_T3)*u_speed; 
    u_steer = (K_omega/(K_d1*(u_speed + eps)*r_torque))*u_omega;  

    u_vert = kz*dz;     % vertical proportional control
    u_vert_orig = u_vert;
    
    u_vert2 = (K_d*u_vert*abs(u_vert) - K_t*(z_offset - z))*(u_vert + v_offset);
    p = [1 k_vert -u_vert2];
    if(u_vert2 > 0),
        p(3) = p(3)/n_up;
    else
        p(3) = -p(3)/n_down;
    end
    z
    u_vert
    p
    rts = sort(real(roots(p)));
    u_vert = rts(2)

    % integrate the dynamics
    dv1dt = (K_T3*u_thrust - K_d1*v1)/(m_total);
    domegadt = (K_T3*u_thrust*r_torque*u_steer - K_omega*omega)/J;
    u_up = 0;
    u_down = 0;
    if u_vert > 0,
        u_up = u_vert;
    else
        u_down = -u_vert;
    end
    
    dzdotdt = ((n_up*u_up*(u_up + k_vert) - n_down*u_down*(u_down + k_vert))/(abs(zdot) + v_offset)...
        - K_d*zdot*abs(zdot) + K_t*(z_offset - z))/m_total;
    
    x = x + v1*cos(th)*dt;
    y = y + v1*sin(th)*dt;
    z = z + zdot*dt;
    zdot = zdot + dzdotdt*dt;
    th = th + omega*dt;
    v1 = v1 + dv1dt*dt;
    omega = omega + domegadt*dt;
    
    U(ix, 1) = u_thrust;
    U(ix, 2) = u_steer;
    U(ix, 3) = u_vert;
    u(ix, 1) = u_speed;
    u(ix, 2) = u_omega;
    u(ix, 3) = u_vert_orig;
    
    if(v1 > vmax)   % saturate forward velocity
        v1 = vmax;
    end
    
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
    ZDOT(ix) = zdot;
    TH(ix) = th;
    V1(ix) = v1;
    V3(ix) = v3;
    OMEGA(ix) = omega;
    
    X_o(ix, 1) = x;
    Y_o(ix, 1) = y;
    Z_o(ix, 1) = z;
end

if(isempty(T_ALL)),
    T_ALL = [];
    X_ALL = [];
    Y_ALL = [];    
    Z_ALL = [];
    GX_ALL = [];
    GY_ALL = [];    
    GZ_ALL = [];    
    ZDOT_ALL = [];
    V1_ALL = [];
    TH_ALL = [];
    OMEGA_ALL = [];
    UT_ALL = [];
    US_ALL = [];
    UV_ALL = []; 
    uT_ALL = [];
    uS_ALL = [];
    uV_ALL = [];    
end

all_figs = findobj('type','figure');
if(ismember(2, all_figs)),
    
    T_ALL = [T_ALL t_now];
    X_ALL = [X_ALL X];
    Y_ALL = [Y_ALL Y];
    Z_ALL = [Z_ALL Z];    
    GX_ALL = [GX_ALL GO_X];
    GY_ALL = [GY_ALL GO_Y];
    GZ_ALL = [GZ_ALL GO_Z];       
    ZDOT_ALL = [ZDOT_ALL ZDOT];
    V1_ALL = [V1_ALL V1];
    TH_ALL = [TH_ALL atan2(sin(TH), cos(TH))];
    OMEGA_ALL = [OMEGA_ALL OMEGA];
    UT_ALL = [UT_ALL U(:, 1)];
    US_ALL = [US_ALL U(:, 2)];
    UV_ALL = [UV_ALL U(:, 3)];    
    uT_ALL = [uT_ALL u(:, 1)];
    uS_ALL = [uS_ALL u(:, 2)];
    uV_ALL = [uV_ALL u(:, 3)]; 

    
    update_fig(H{1}, T_ALL, [X_ALL; GX_ALL]);
    update_fig(H{2}, T_ALL, [Y_ALL; GY_ALL]);
    update_fig(H{3}, T_ALL, [Z_ALL; GZ_ALL]);
    update_fig(H{4}, T_ALL, V1_ALL);
    update_fig(H{5}, T_ALL, ZDOT_ALL);
    update_fig(H{6}, T_ALL, TH_ALL);
    update_fig(H{7}, T_ALL, OMEGA_ALL);
    
    update_fig(H{8}, T_ALL, UT_ALL);    
    update_fig(H{9}, T_ALL, US_ALL);    
    update_fig(H{10}, T_ALL, UV_ALL);   
    update_fig(H{11}, T_ALL, uT_ALL);
    update_fig(H{12}, T_ALL, uS_ALL);    
    update_fig(H{13}, T_ALL, uV_ALL); 
else
    % f2: x y z, zdot, v1, th, omega
    % f3: U_T, U_S, U_V
    H = cell(0);
    
    figure(2);      
    subplot(3, 3, 1)
    H{1} = plot(t_now, [X; GO_X]);
    set(H{1}(5 : end), 'LineStyle', '--');
    set(H{1}, {'Color'}, {'r', 'g', 'b', 'k', 'r', 'g', 'b', 'k'}')
    ylabel('X');

    subplot(3, 3, 2)
    H{2} = plot(t_now, [Y; GO_Y]);
    set(H{2}(5 : end), 'LineStyle', '--');
    set(H{2}, {'Color'}, {'r', 'g', 'b', 'k', 'r', 'g', 'b', 'k'}')    
    ylabel('Y');
    
    subplot(3, 3, 3)
    H{3} = plot(t_now, [Z; GO_Z]);
    set(H{3}(5 : end), 'LineStyle', '--');
    set(H{3}, {'Color'}, {'r', 'g', 'b', 'k', 'r', 'g', 'b', 'k'}')    
    ylabel('Z');
    
    subplot(3, 3, 4)
    H{4} = plot(t_now, V1);
    ylabel('V1');
    
    subplot(3,3,5)
    H{5} = plot(t_now, ZDOT);
    ylabel('Zdot');
    
    subplot(3,3,6)
    H{6} = plot(t_now, TH);
    ylabel('Theta');
    
    subplot(3,3,7)
    H{7} = plot(t_now, OMEGA);
    ylabel('Omega');
    
    figure(3)
    subplot(3,2,1)
    H{8} = plot(t_now, U(:, 1));
    ylabel('Thrust')
    
    subplot(3,2,3)
    H{9} = plot(t_now, U(:, 2));
    ylabel('Steering')
    
    subplot(3,2,5)
    H{10} = plot(t_now, U(:, 3));
    ylabel('Vertical Thrust')
    
    subplot(3,2,2)
    H{11} = plot(t_now, u(:, 1));
    ylabel('Speed Cmd')
    
    subplot(3,2,4)
    H{12} = plot(t_now, u(:, 2));
    ylabel('Omega Cmd')
    
    subplot(3,2,6)
    H{13} = plot(t_now, u(:, 3));
    ylabel('Vertical Cmd')
    
    T_ALL = [t_now];
    X_ALL = X;
    Y_ALL = Y;
    Z_ALL = Z;  
    GX_ALL = GO_X;
    GY_ALL = GO_Y;
    GZ_ALL = GO_Z;    
    ZDOT_ALL = [ZDOT];
    V1_ALL = V1;
    TH_ALL = atan2(sin(TH), cos(TH));
    OMEGA_ALL = OMEGA;
    UT_ALL = U(:, 1);
    US_ALL = U(:, 2);
    UV_ALL = U(:, 3);
    uT_ALL = U(:, 1);
    uS_ALL = U(:, 2);
    uV_ALL = U(:, 3);    
end

function update_fig(h, t, y)

set(h, {'XData'}, num2cell(repmat(t, [size(y, 1) 1]), 2), {'YData'}, num2cell(y, 2));