function rW = imageToWorld(u, v, f, c, R, T, d, h)

n1 = 1.333;
n2 = 1.0;

x = (u - c(1))/f(1);
y = (v - c(2))/f(2);

r = [x; y; 1];

CW = -R'*T;

Rt = R';

if d <= 0,
    t = (-h-d)/(Rt(3,:)*r);
    rW = CW + t*R'*r;
    return;
end
    
%C(3) - h - d - C(3)
t = (-h)/(Rt(3,:)*r);
S = t*R'*r;

phi = atan2(S(2),S(1));
rS = sqrt(S(1)^2 + S(2)^2);

rP = rS + n2*rS*d/sqrt(n1^2*(h^2 + rS^2) - n2^2*rS^2);

rW = CW + [rP*cos(phi); rP*sin(phi); -(h+d)];