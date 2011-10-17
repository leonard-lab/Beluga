% UKF   Unscented Kalman Filter for nonlinear dynamic systems
% [x, P] = ukf(f,x,P,h,z,Q,R) returns state estimate, x and state covariance, P
% for nonlinear dynamic system:
%           x_k+1 = f(x_k,u_k,v_k)
%           z_k   = h(x_k,n_k)
% where v ~ N(0,Q) meaning v is gaussian process disturbance covariance Q
%       n ~ N(0,R) meaning n is gaussian measurement noise with covariance R
% Inputs:   fstate: function handle for f(x,u,v)
%           x: "a priori" state estimate
%           P: "a priori" estimated state covariance
%           hmeas: fanction handle for h(x,n)
%           z: current measurement
%           Q: process noise covariance
%           R: measurement noise covariance
%           u: system control input
% Output:   x: "a posteriori" state estimate
%           P: "a posteriori" state covariance
%



function [x,P]=ukf_aug(fstate,x,P,hmeas,z,Q,R,u)



n=numel(x);                                 %numer of states
m=numel(z);                                 %numer of measurements
Lv=size(Q,1);                          %number of process noise inputs
Ln=size(R,1);                          %number of measurement noise inputs

L=n+Lv+Ln ;                                %number of augmented states

alpha=1e-3;                                 %default, tunable
k=0;                                       %default, tunable
beta=2;                                     %default, tunable
lambda=alpha^2*(L+k)-L;                    %scaling factor
c=L+lambda;                                 %scaling factor
Wm=[lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
Wc=Wm;
Wc(1)=Wc(1)+(1-alpha^2+beta);               %weights for covariance
c=sqrt(c);

xa=[x ; zeros(Lv,1) ; zeros(Ln,1)]; 

Pa=[P zeros(n,Lv+Ln) ; zeros(Lv,n) Q zeros(Lv,Ln) ; zeros(Ln,n+Lv) R];


Xa=sigmas(xa,Pa,c);                            %sigma points around augmented state matrix

%breaking down augmented vector
X=Xa(1:n,:); 
Xv=Xa(n+1:n+Lv, :);
Xn=Xa(n+Lv+1:end, :);


[x1,X1,P1,X2]=ut_state(fstate,X,Xv,Wm,Wc,u)          %unscented transformation of process

[z1,P2,Z2]=ut_meas(hmeas,X1,Xn,Wm,Wc,R);       %unscented transformation of measurments

P12=X2*diag(Wc)*Z2'                        %transformed cross-covariance

if isnan(sum(z))
    x = x1 ;
else
    
    R = chol(P2);
    condNum = cond(P2)
    K = (P12/R)/R'                             % Filter gain. K=P12*inv(P2)
    x=x1+(K*(z-z1))             %state update
    %note above that dropped measurements (NaN) are accounted for
    P=P1-P12*K'                                %covariance update
    condP1 = cond(P1)
    condUpdate = cond(P12*K')
    condP = cond(P)
end
















end