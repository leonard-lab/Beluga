%generate a random smooth signal using fourier components
function [fn] = gen_rand_smooth(t, base_freq, max_freq, n_comp)

if nargin < 2,
    base_freq = 2*pi*0.1;
end

if nargin < 3,
    max_freq = 3*base_freq;
end

if nargin < 4,
    n_comp = 12;
end

omega = base_freq + (max_freq - base_freq)*base_freq*rand(n_comp, 1);
an=randn(1,n_comp) ; bn=randn(1,n_comp) ; %magnitudes
fn=0 ;
for j =1 :1: n_comp
    fn = fn + an(j)*sin(omega(j)*t) + bn(j)*cos(omega(j)*t) ;
end
end