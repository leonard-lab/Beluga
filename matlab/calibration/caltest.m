clear all;  close all;

I = imread('BGD.bmp');

B = rgb2gray(I);
H = rgb2hsv(I);

M = (H(:,:,3) < 0.34);
M = medfilt2(M, [5 5]);

L = bwlabel(M);
L_most = mode(reshape(L, [numel(L) 1]));

MM = (L == L_most);
MM = imfill(MM, 'holes');

MB = immultiply(B, MM);
MH = immultiply(H(:,:,1), MM);

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

xc = y(1) + mean(mean(b.*repmat(y', [1 length(x)])));a
yc = x(1) + mean(mean(b.*repmat(x, [length(y) 1])));

plot(xc, yc, 'ro')


figure(3)


disp('Calculating Hough transform')
edge_ignore = 5;
[H,T,R] = hough(E);%([edge_ignore : end - edge_ignore, edge_ignore : end-edge_ignore]));
imshow(H,[],'XData',T,'YData',R,...
            'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;


disp('Finding Hough peaks')
num_peaks = 25;
peak_thresh = ceil(0.1*max(H(:)));
P  = houghpeaks(H,num_peaks, 'threshold', peak_thresh);

x = T(P(:,2)); y = R(P(:,1));
plot(x,y,'s','color','white');

% Find lines and plot them
disp('Finding lines')
gap = 35;
min_length = 50;
lines = houghlines(E,T,R,P, 'FillGap',gap,'MinLength',min_length);
figure, imshow(MB), hold on

lines_in = lines;
lines = [];

all_lengths = [];
D = [];
C = flipud(repmat(Cent_Guess', [1 2]));
d_max = 30;
A = [];  B = [];
for ix = 1 : length(lines_in),
    len = norm(lines_in(ix).point1 - lines_in(ix).point2);
    all_lengths = [all_lengths; len];
    
    r1 = lines_in(ix).point1';
    r2 = lines_in(ix).point2';
    d = min(sqrt(sum(([r1 r2] - C).^2)));
    D = [D; d];

    if(d > d_max)
        continue;
    end

    lines = [lines; lines_in(ix)];
    
    dr = lines_in(ix).point2' - lines_in(ix).point1';
    m = dr(2)/dr(1);
    b = lines_in(ix).point2(2) - m*lines_in(ix).point2(1);
    A = [A; m -1];
    B = [B; -b];
end

for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth', 1,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',1,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',1,'Color','red');

end

TANK_CENTER = (A'*A)\(A'*B);  % = inv(A'*A)*(A'*B);
plot(TANK_CENTER(1), TANK_CENTER(2), 'ro', 'LineWidth', 2);

tank_search_window_width = 50;
min_r = 20;
max_r = 40;

RA = [];
tx = TANK_CENTER(1);  ty = TANK_CENTER(2);

for x = round(tx) - tank_search_window_width : round(tx) + tank_search_window_width,
    for y = round(ty) - tank_search_window_width : round(ty) + tank_search_window_width,
        r = sqrt((tx -x)^2 + (ty - y)^2);
        if(E(x,y)),
            RA = [RA; r];
        end
    end
end

[n, b] = hist(RA, [min_r : 0.25 : max_r]);
n = n(2 : end-1);
b = b(2 : end-1);
[~, mx] = max(n);
TANK_CENTER_RADIUS = b(mx);

C = (TANK_CENTER(1) + i*TANK_CENTER(2)) + TANK_CENTER_RADIUS*exp(i*linspace(0,2*pi));
plot(C, 'r-')

