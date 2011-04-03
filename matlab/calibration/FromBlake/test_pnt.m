% Extract information from initial image and create template rectified image
function [dst] = test_pnt(px,py)

load values kc cc fc alpha_c
k = kc; c = cc; f = fc; alpha = alpha_c; R = eye(3);

% compute the KK matrix transform to pixel coordinates
KK = [f(1) alpha*f(1) c(1);0 f(2) c(2) ; 0 0 1];
          
% creates an array of pixel locations, and adds a row of 1s for the
% inverse from pixel coordinates to normalized coortinate points (KK)
rays = inv(KK)*[px;py;ones(1,length(px))];

% Rotation (or affine transformation) and normalization by third column:
rays2 = R'*rays;
x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)]; % points normalized by third vector component

% Add distortion: this computes a matrix xd of the points of the distorted 
% set of points corresponding to the undistorted coordinates x (not pixel locations)
xd = distort(x,k);

% Reconvert in pixels (essentially by applying KK matrix):
px2 = f(1)*(xd(1,:)+alpha*xd(2,:))+c(1);
py2 = f(2)*xd(2,:)+c(2);

dst = [px2;py2];

% fprintf(1,['x-distorted = ' num2str(px2) ' y-distorted = ' ...
%                 num2str(py2) '\n']);
end