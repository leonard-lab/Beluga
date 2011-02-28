%%%     Computation of Coordinate Transformation    %%%

% consider 3-D point P defined by [Xw,Yw,Zw] in the real world reference
% frame.  Let [Xc,Yc,Zc] be the coordinated of P in the camera reference
% frame (with Zc > 0 if P is visible).  The origin of the camera frame is
% in the center of the projection and the Z axis is the optical axis.

% coordinat transform: [Xc Yc Zc]' = R*[Xw Yw Zw]' + T

% assuming that radial distortions can be neglected, we can write the iage
% of [Xc Yc Zc] ain the image referece frame as (in pixel coordinates):
%   xim = -f/sx * Xc/Zc + ox
%   yim = -f/sy * Yc/Zc + oy
% WHERE: focal length (f), horizontal and vertical effective pixel size (sx
% and sy), coordinates of the image center (ox and oy)
% LET: fx = f/sx, and alpha = sy/sx >> four intrinsic parameters
%   ox,oy,fx,alpha

% Two steps: 1. assuming the coordinates of the image center are known,
% estimate all the remaining parameter; 2. find the coordinates of the
% image center

% we consider the translated coordinates (x,y) = (xim - ox, yim - oy),
% which is equivalent to assuming the image center is the image origin

%%%     ASSUMPTIONS AND PROBLEM STATEMENT   %%%
% assuming that the location of the image center (ox,oy) is known, and that
% hte radial distortion can be neglected, estimate fx, alpha, R and T from
% image points (xi,yi), i = 1...N, projections of N known world points
% [Xwi,Ywi,Zwi]

% therefore we should have two matrices of corresponding points im_pts(2,N)
% and wrld_pts(3,N) where corresponding columns match

A = zeros(N,8);
for i = 1:N
    A(i,1) = im_pts(1,i) * wrld_pts(1,i);
    A(i,2) = im_pts(1,i) * wrld_pts(2,i);
    A(i,3) = im_pts(1,i) * wrld_pts(3,i);
    A(i,4) = im_pts(1,i);
    A(i,5) = - im_pts(2,i) * wrld_pts(1,i);
    A(i,6) = - im_pts(2,i) * wrld_pts(2,i);
    A(i,7) = - im_pts(2,i) * wrld_pts(3,i);
    A(i,8) = - im_pts(2,i);   
end

if rank(A) ~= 7
    fprintf(1,'The A matrix has rank =/= 7, the solution will be trivial.\n')
    return;
end

% compute the vector vbar = gamma * (r21,r22,r23,Ty,alpha*r11,alpha*r12,
% alpha*r13,alpha*Tx), where A*v = 0 and vbar = gamma*v (unkonwn scaling) 
v = null(A);

gamma = sqrt(v(1)^2 + v(2)^2 + v(3)^2);
alpha = sqrt(v(5)^2 + v(6)^2 + v(7)^2)/gamma;

R = zeros(3);
R(1,1) = v(5);  R(1,2) = v(6);  R(1,3) = v(7);  R(1,:) = R(1,:)/alpha;
R(2,1) = v(1);  R(2,2) = v(2);  R(2,3) = V(3);  R(1:2,:) = R(1:2,:)/gamma;
R(3,:) = cross(R(1,:),R(2,:));

T = zeros(3,1); T(1) = v(8)/alpha; T(2) = v(4); T = T/gamma;

% use SVD to ensure orthogonality of R matrix
[U,S,V] = svd(R);
R = U*eye(3)*V';

% check point 1 with R and T to determine sign change
if im_pts(1,1)*(R(1,1)*wrld_pts(1,1) + R(1,2)*wrld_pts(2,1)...
        + R(1,3)*wrld_pts(3,1) + T(1))>0
    R(1:2,:) = -R(1:2,:);
    T = -T;
end

% compute remaining parameters

A2 = zeros(N,2);
for i = 1:N
    A(i,1) = im_pts(1,i);
    A(i,2) = (R(1,1)*wrld_pts(1,i) + R(1,2)*wrld_pts(2,i) + ...
        R(1,3)*wrld_pts(3,i) + T(1));
end

b = zeros(N,1);
for i = 1:N
    B(i,1) = -im_pts(1,i)*(R(3,1)*wrld_pts(1,i) + R(3,2)*wrld_pts(2,i) + ...
        R(3,3)*wrld_pts(3,i));
end

temp = (A2'*A2)\A2'*b;

T(3,1) = temp(1);
fx = temp(2);
