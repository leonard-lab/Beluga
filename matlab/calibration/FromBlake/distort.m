function [xd] = distort(x,k)

% Complete the distortion vector if you are using the simple distortion model:
length_k = length(k);
if length_k <5 ,
    k = [k ; zeros(5-length_k,1)];
end;

[m,n] = size(x);

% Add distortion:
r2 = x(1,:).^2 + x(2,:).^2;
r4 = r2.^2;
r6 = r2.^3;

% Radial distortion:
cdist = 1 + k(1) * r2 + k(2) * r4 + k(5) * r6;
xd1 = x .* (ones(2,1)*cdist);

% Tangential distortion:
a1 = 2.*x(1,:).*x(2,:);
a2 = r2 + 2*x(1,:).^2;
a3 = r2 + 2*x(2,:).^2;

delta_x = [k(3)*a1 + k(4)*a2 ;
           k(3)*a3 + k(4)*a1];

% Total distortion:
xd = xd1 + delta_x;


return;