function [rs, fmin] = testNewtonRS()

n1 = 1.333;
n2 = 1.0;

h = 3;
d = 2;

rp = 0.5;

rs = 0.8*rp;

thresh = 1e-4;
ixmax = 10;

RS = linspace(0, rp);
figure(1)
clf
plot(RS, S(rp, RS, n1, n2, d, h), 'k-');
grid on
hold on

for ix = 1 : ixmax,
    f = S(rp, rs, n1, n2, d, h);
    
    if abs(f) < thresh,
        break;
    end
    
    fp = Sp(rp, rs, n1, n2, d, h)
    
    if abs(fp) < thresh,
        fp = -1;
    end
    
    plot(rs, f, 'go')
    rsp = rs;
    rs = rs - f/fp;
    plot(rs, 0, 'gs', [rsp rs], [f 0], 'g-')
    pause
    
    if rs > rp,
        rs = rp;
    end
    if rs < 0,
        rs = 0;
    end
end

fmin = f;


function fo = S(rp, rs, n1, n2, d, h)

fo = (n1^2*(h^2 + rs.^2)-n2^2*rs.^2).*(rp-rs).^2-n2^2*d^2*rs.^2;

function fpo = Sp(rp, rs, n1, n2, d, h)

fpo = (2*n1^2*rs - 2*n2^2*rs)*(rp-rs)^2 - 2*(n1^2*(h^2+rs^2)-n2^2*rs^2)*(rp-rs) - 2*n2^2*d^2*rs;
%2*rs.*((n1^2-n2^2)*(rp-rs).^2 - 2*n2^2*d^2) - 2*(n1^2*(h^2+rs.^2)-n2^2*rs.^2).*(rp-rs);