function rS = solveForRS(rP, d, h)

rS = 0;

n1 = 1.0;
n2 = 1.333;

RS = linspace(0, rP, 1e3);

LHS = n2^2*(rP-RS).^2.*(RS.^2+h^2);
RHS = n1^2*RS.^2.*(d^2 + (rP-RS).^2);

[~, ix] = min(abs(LHS-RHS));

rS = RS(ix);