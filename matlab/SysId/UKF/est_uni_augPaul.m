%Implements the augmented UKF for unicycle dynamics and produces output
%plots to check quality of convergence refer to ukf_aug for more details%about the nonlinear model considered.



clear all;  close all;


%timing paramenters
dt=0.05 ;
tf = 20 ;
t=0:dt:tf ;
N=length(t) ;

n=5;      %number of state
m=3;      %number of outputs

%random control input
u = rand(2,length(t)) ;
u(1,:) = gen_rand_smooth(t, 1); %40*ma(u(1,:),15);
u(2,:) = gen_rand_smooth(t, 1);

u(1, :) = u(1, :) - min(u(1,:));
u(1, :) = 45*( u(1, :)/max(u(1,:)) );
u(2, :) = u(2, :)/(max(abs(u(2,:))));

% q=.2;    %std of process
% r=.6;    %std of measurement
% 
% Q=q^2*eye(n); % covariance of process
% R=r^2*eye(m);        % covariance of measurement

sigma_pos = 0.01;
sigma_speed = 0.05;
sigma_head = 0.2;
sigma_omega = 0.5;
Q = diag([sigma_pos sigma_speed sigma_pos sigma_head sigma_omega].^2);

sigma_pos_meas = 0.05;
sigma_hdg_meas = 10*pi/180;
R = diag([sigma_pos_meas sigma_pos_meas sigma_hdg_meas].^2);

J = 2.5 ;
Kt = 0.016*0.50*5.6*23;
r1 = 0.35;
Komega = 2.5+4.5;
Kd1 = 60*0.75;
m0 = 7.4;
m1 = 12*0.5;


% x = [ x speed y heading omega ] [ m m/sec m rad rad/sec]
% u = [ u_speed u_heading ] [
f=@(x,u,v)x+v+dt*[(x(2)*cos(x(4)));
    ( ( ( Kt*(u(1)) )*1 - Kd1*x(2) )/(m0 + m1) );
    ( x(2)*sin(x(4)) );
    ( x(5) );
    ( (r1*( Kt*u(1) )*(u(2)) - Komega*x(5) )/J );
    ];  % nonlinear state equations


h=@(x,n) n+[x(1); x(3); x(4)];                               % measurement equation

%f=@(x,u,v)x+dt*[(u(1)+v(1))*cos(x(3));(u(1)+v(1))*sin(x(3));(u(2)+v(2))];  % nonlinear state equations
%h=@(x,n) [x(1)+n(1);x(2)+n(2)];                               % measurement equation


%simulation of dynamics
x0=[0; 0; 0; 0; 0] ; %initial cond

X=zeros(5,length(t)) ; X(:,1) =x0 ; %simulated actual trajectory

Xhat=zeros(5,length(t)) ;  %estimated trajectory

Z = zeros(3,length(t)) ; %measurements

Uactual = 0*u ; %save space for actual input

P=eye(5) ; %initial covariance estimate


for j=2:1:length(t)
    Z(:,j-1) = h(X(:,j-1),chol(R)*randn(m,1)) ;  %measurement matrix
    
%     if rand()>.90
%         Z(:,j-1) = NaN*ones(m,1) ;         %simulate dropped measurements
%     end
    
    ud = diag([5 0.1])*randn(2, 1);
    d = chol(Q)*randn(n,1);                     %input disturbance
    Uactual(:,j-1) = u(:,j-1) + ud;         %save actual input
    X(:,j) = f(X(:,j-1) , u(:,j-1) , d) ;  %update actual state
    [Xhat(:,j),P]=ukf_aug(f,Xhat(:,j-1),P,h,Z(:,j-1),Q,R,u(:,j-1)) ; %update state estimate
    
    
    
end


%plot results

subplot 311
plot(t, X(1,:)) ; hold on ; plot(t, Xhat(1,:),'r', t, Z(1,:),'k');
legend('Actual x','Estimate x_{hat}','Measured z1');

tn = t(isnan(Z(1,:)));
plot(tn, zeros(size(tn)), 'o');

subplot 312
plot(t, X(3,:)) ; hold on ; plot(t, Xhat(3,:),'r', t, Z(2,:),'k');
legend('Actual y','Estimate y_{hat}','Measured z2');

subplot 313
plot(t, X(4,:)) ; hold on ; plot(t, Xhat(4,:),'r') ;
legend('Actual \theta','Estimate \theta_{hat}');


figure ;
subplot 211
plot(t,u(1,:)); hold on ; plot(t,Uactual(1,:),'r') ; legend('u1 desired','u1 actual');
subplot 212
plot(t,u(2,:)); hold on ; plot(t,Uactual(2,:),'r') ; legend('u2 desired','u2 actual');

%simplot(X,Xhat) ;  %simulation





