function [angles, v] = FindSeamAnglesAtRadiusFromCenter(Image, radius, center, guess)
 
% Image = MV_smooth;
% radius = Rs(1);
% center = TANK_CENTER';
% guess = g;

angles = [];

radius = round(radius);

search_annulus_half = 8;
MAX_ITER = 10;
MIN_CONTRAST = 0.15;
NUM_BINS = 500;
MIN_ANGLE_DIFF = 10*pi/180;

[frame_height, frame_width] = size(Image);

se_a = strel('disk', radius + search_annulus_half, 0);
se_b = strel('disk', radius - search_annulus_half, 0);

a = se_a.getnhood;
b = zeros(size(a));

ds = 2*search_annulus_half;
b([ds+1 : end-ds],[ds+1 : end-ds]) = se_b.getnhood;

b = b - a;

se_c = strel('arbitrary', b);
search_px = se_c.getneighbors;

search_px = search_px + repmat(round(center), [length(search_px) 1]); 

S_TH = [];
S_THa = [];
S_Va = [];
S_V = [];

w = (15)/radius;
w = min([w (1/3)*2*pi/14]);
if radius < 65,
    w = 0.4*2*pi/14;
end

for ix = 1 : length(search_px),

    x = search_px(ix, 1);
    y = search_px(ix, 2);
    
    if(y < 1 || y > frame_height || x < 1 || x > frame_width)
        continue
    end
    
    %r = sqrt((tx-x)^2 + (ty-y)^2);
    
    th = atan2(y - center(2), x - center(1));
    
    S_THa = [S_THa; th];
    S_Va = [S_Va; Image(y,x)];
    
    if min(abs(guess - th)) > w,
        continue;
    end
    
    S_TH = [S_TH; th];
    S_V = [S_V; Image(y, x)];

end

[S_THa, ix] = sort(S_THa);
S_Va = S_Va(ix);

% detrending S_V - find best-fit sinusoidal trend
%  - a sinusoidal trend is consistent with the illumination changing
%    roughly linearly across the tank
[S_TH, ix] = sort(S_TH);
S_V = S_V(ix);

%return

%S_V = smooth(S_V, 2*floor(length(S_V)/14/5/2) + 1);

a0 = mean(S_V);
Phi = [sin(S_TH) cos(S_TH) sin(2*S_TH) cos(2*S_TH)];
Y = [S_V - a0];
A = Phi\Y;
S_V = S_V - (a0 + A(1)*sin(S_TH) + A(2)*cos(S_TH) ...
    + A(3)*sin(2*S_TH) + A(4)*cos(2*S_TH));
% remove any remaining linear trend
S_V = detrend(S_V);

if max(S_V) - min(S_V) < MIN_CONTRAST,
    return
end

b = linspace(-pi, pi, NUM_BINS+1);
b = 0.5*(b([1 : end-1]) + b([2 : end]));
db = b(2) - b(1);
v = zeros(size(b));

for bx = 1 :  length(b),
    ix = find(abs(S_TH - b(bx)) < db);
    if numel(ix),
        v(bx) = min(S_V(ix));
    else
        v(bx) = NaN;
    end
end

b = b(isfinite(v));
v = v(isfinite(v));
d = zeros(size(b));
mv = zeros(size(b));

E = [1 0];
for ix = 1 : length(b)-1,
    if b(ix+1) - b(ix) > mean(diff(b)),
        E(end, 2) = ix;
        E = [E; ix+1 0];
    end
end
E(end, 2) = length(b);

s = zeros(length(E), 1);
for ex = 1 : length(E),
    [~, ix] = min(v([E(ex, 1) : E(ex, 2)]));
    s(ex) = b(ix + E(ex, 1) - 1);
end

angles = sort(s);

% return
% 
% w = floor(NUM_BINS/14/2);
% ix = [-w : w];
% for jx = 1 : length(b),
%     IX = jx + ix;
%     IX = IX(IX > 0);
%     IX = IX(IX < length(b));
%     
%     mv(jx) = min(v(IX));
%     if v(jx) == min(v(IX)),
%         d(jx) = 1;
%     end
%     if jx > 1 && (v(jx - 1) == min(v(IX))),
%         d(jx) = 1;
%     end
%     if jx < length(b) && (v(jx+1) == min(v(IX))),
%         d(jx) = 1;
%     end
% end
% 
% s = b(d > 0);
% s0 = s;
% 
% while min(diff(s)) < MIN_ANGLE_DIFF,
%     s_o = [];
%     jx = 1;
%     while jx < length(s)-1,
%         if s(jx+1) - s(jx) < MIN_ANGLE_DIFF,
%             s_o = [s_o 0.5*(s(jx+1)+s(jx))];
%             jx = jx + 1;
%         else
%             s_o = [s_o s(jx)];
%         end
%         jx = jx + 1;
%     end
%     s = [s_o s(end)];
% end
% 
% if abs(s(1)+ 2*pi - s(end)) < MIN_ANGLE_DIFF,
%     s(end) = 0.5*(s(1)+2*pi+s(end));
%     s = s([2 : end]);
% end
% 
% %g = exp(i*s);
% %angles = atan2(real(g), imag(g));
% angles = sort(s);
