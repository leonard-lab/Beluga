clear all;  close all;

I = imread('BGD.bmp');

B = rgb2gray(I);
HSV = rgb2hsv(I);
H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);

M = (V < 0.34);
M = medfilt2(M, [5 5]);

L = bwlabel(M);
L_most = mode(reshape(L, [numel(L) 1]));

MM = (L == L_most);
MM = imfill(MM, 'holes');

MB = immultiply(B, MM);
MH = immultiply(H, MM);

h = fspecial('gaussian', 3);
MBF = imfilter(MB, h);

E = edge(MBF, 'canny');

figure(1)
imagesc(MM, [0 1])
colormap gray

figure(2)
imagesc(E);
axis image
colormap gray
hold on

Cent_Guess = fliplr([234 77]);

W = 35;

x = [Cent_Guess(1) - W : Cent_Guess(1) + W];
y = [Cent_Guess(2) - W  : Cent_Guess(2) + W];
b = E(x, y);

xc = y(1) + mean(mean(b.*repmat(y', [1 length(x)])));
yc = x(1) + mean(mean(b.*repmat(x, [length(y) 1])));

plot(xc, yc, 'ro')


figure(3)


disp('Calculating Hough transform')
edge_ignore = 5;
se1 = strel('disk', 4);
se2 = strel('disk', 5);
%EH = imerode(imdilate(E, se1), se2);
EH = E;
[Ho,T,R] = hough(EH);%([edge_ignore : end - edge_ignore, edge_ignore : end-edge_ignore]));
imshow(Ho,[],'XData',T,'YData',R,...
            'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;


disp('Finding Hough peaks')
num_peaks = 40;
peak_thresh = ceil(0.1*max(Ho(:)));
P  = houghpeaks(Ho,num_peaks, 'threshold', peak_thresh);

x = T(P(:,2)); y = R(P(:,1));
plot(x,y,'s','color','white');

% Find lines and plot them
disp('Finding lines')
gap = 10;
min_length = 50;
lines = houghlines(E,T,R,P, 'FillGap',gap,'MinLength',min_length);
figure, imshow(MB), hold on

lines_in = lines;
lines = [];

all_lengths = [];
D = [];
C = flipud(repmat(Cent_Guess', [1 2]));
d_max = 40;
A = [];  B = [];
Phi = [];  Y = [];
for ix = 1 : length(lines_in),
    len = norm(lines_in(ix).point1 - lines_in(ix).point2);
    all_lengths = [all_lengths; len];
    
    r1 = lines_in(ix).point1';
    r2 = lines_in(ix).point2';
    d = min(sqrt(sum(([r1 r2] - C).^2)));
    c = sqrt(sum( (0.5*(r1 + r2)-C(:,1)).^2));
    D = [D; d];

    if(d > d_max || c < d)
        continue;
    end

    lines = [lines; lines_in(ix)];
    
    dr = lines_in(ix).point2' - lines_in(ix).point1';
    m = dr(2)/dr(1);
    b = lines_in(ix).point2(2) - m*lines_in(ix).point2(1);
    A = [A; m -1];
    B = [B; -b];
    
    xk1 = lines_in(ix).point1(1);
    xk2 = lines_in(ix).point2(1);
    yk1 = lines_in(ix).point1(2);
    yk2 = lines_in(ix).point2(2);
    
    dyk = yk2 - yk1;
    dxk = xk2 - xk1;
    drk = sqrt(dxk^2 + dyk^2);
    
    Phi = [Phi; dyk/drk -dxk/drk];
    Y = [Y; -(1/drk)*(dxk*yk1-dyk*xk1)];
    
end

for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth', 1,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',1,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',1,'Color','red');

end

TANK_CENTER2 = (A\B);  % = inv(A'*A)*(A'*B);
TANK_CENTER = (Phi\Y);
plot(TANK_CENTER(1), TANK_CENTER(2), 'ro', 'LineWidth', 2, 'MarkerSize', 10);
plot(TANK_CENTER2(1), TANK_CENTER2(2), 'rs', 'LineWidth', 2, 'MarkerSize', 10);


tank_search_window_width = 50;
min_r = 20;
max_r = 40;

RA = [];
tx = TANK_CENTER(1);  ty = TANK_CENTER(2);

seam_search_radius = 45;
S_TH = [];
S_V = [];
for x = round(tx) - tank_search_window_width : round(tx) + tank_search_window_width,
    for y = round(ty) - tank_search_window_width : round(ty) + tank_search_window_width,
        r = sqrt((tx -x)^2 + (ty - y)^2);
        if(E(y,x)),
            RA = [RA; r];
        end
        
        if abs(round(r) - seam_search_radius) < 5,
            S_TH = [S_TH; atan2(x - tx, y - ty)];
            S_V = [S_V; V(y,x)];
        end
    end
end

[n, b] = hist(RA, [min_r : 0.25 : max_r]);
n = n(2 : end-1);
b = b(2 : end-1);
[~, mx] = max(n);
TANK_CENTER_RADIUS = b(mx);

C = (TANK_CENTER(1) + i*TANK_CENTER(2)) + TANK_CENTER_RADIUS*exp(i*linspace(0,2*pi));
CS = (TANK_CENTER(1) + i*TANK_CENTER(2)) + seam_search_radius*exp(i*linspace(0,2*pi));
plot(C, 'r-')
plot(CS, 'g-')

S_b = S_V < (mean(S_V) - (1/3)*(mean(S_V) - min(S_V)));
S_t = sort(S_TH(S_b > 0));
S_THo = S_TH;
[S_TH, ix] = sort(S_TH);
S_b = S_b(ix);
splits = [-180 : 180/7 : 180];
d = [diff(S_t)' 0];
dL = [];
t_d = 0.5;
while length(dL) ~= 13,
    dL = find(d > t_d*max(d));
    if(length(dL) < 14),
        t_d = t_d + 0.1;
    else
        t_d = t_d - 0.1;
    end
end

SEAM_ANGLES = zeros(1,14);
SEAM_ANGLES(1) = mean(S_t(1 : dL(1)));
for ix = 1 : 12,
    SEAM_ANGLES(ix+1) = mean(S_t(dL(ix)+1 : dL(ix + 1)));
end
SEAM_ANGLES(end) = mean(S_t(dL(end) : end));

CS = (seam_search_radius*exp(i*SEAM_ANGLES));
CS = TANK_CENTER(1) + i*TANK_CENTER(2) + imag(CS) + i*real(CS);
plot(CS, 'ro')

h_unsharp = fspecial('unsharp');
MBS = imfilter(MB, h_unsharp);

s_px = cell(14,1);

for ix = 1 : 14,
    a = SEAM_ANGLES(ix);
    px = TANK_CENTER(1) + seam_search_radius*sin(a);
    py = TANK_CENTER(2) + seam_search_radius*cos(a);
    p = [px py];
    
    s_px{ix} = follow_seam(TANK_CENTER, a, p, MB); 
end

figure
subplot(3,1,1)
plot(S_THo*180/pi, S_V, '.', repmat(SEAM_ANGLES*180/pi, [2 1]), [zeros(1, 14); ones(1, 14)], '-')
subplot(3,1,2)
plot(S_TH*180/pi, S_b, '.', repmat(SEAM_ANGLES*180/pi, [2 1]), [zeros(1, 14); ones(1, 14)], '-')
subplot(3,1,3)
stem(S_t*180/pi, d*180/pi)
hold on
plot(repmat(SEAM_ANGLES*180/pi, [2 1]), [zeros(1, 14); 30*ones(1, 14)], '-')
