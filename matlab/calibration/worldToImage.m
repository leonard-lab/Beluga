function rI = worldToImage(rW, f, c, R, T, h)

CW = -R'*T;


d = CW(3) - h - rW(3);

if d > 0,
    phi = atan2(rW(2)-CW(2), rW(1)-CW(1));
    rP = sqrt( (rW(1)-CW(1))^2 + (rW(2)-CW(2))^2);
    rS = solveForRS(rP, d, h);
    S = CW + [rS*cos(phi); rS*sin(phi); -h];
else
    S = rW;
end


X = R*S + T;

if X(3) == 0,
    X(3) = 1;
end

x = X(1)/X(3);
y = X(2)/X(3);

u = f(1)*x + c(1);
v = f(2)*y + c(2);

rI = [u; v];