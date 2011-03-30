clear all;  close all;

Q = 1;

im_file = sprintf('Q%d.bmp', Q);
cal_file = sprintf('SnellCalib-Q%d.dat', Q);

I = imread(im_file);
[nrows, ncols] = size(I);

[~, ~, alpha_c, kc, R, T, lines] = ReadCalibrationData(cal_file);
C = -R'*T;

cc = [320 240];
fc = [472 472];

imshow(I)
hold on

% water depth est 7.5'
w = 2.286;
h = C(3)-w;

rW = [0; 0; 0];
rI = worldToImage(rW, fc, cc, R, T, h);
plot(rI(1), nrows - rI(2), 'gs')

rW = [1; 0; 0];
rI = worldToImage(rW, fc, cc, R, T, h);
plot(rI(1), nrows - rI(2), 'gs')

rW = [0; 1; 0];
rI = worldToImage(rW, fc, cc, R, T, h);
plot(rI(1), nrows - rI(2), 'gs')

rW = [0; 0; 1.1*w];
rI = worldToImage(rW, fc, cc, R, T, h);
plot(rI(1), nrows - rI(2), 'bo')

u = 330;
v = nrows - 240;
d = 1;
plot(u, v, 'bo')
rW = imageToWorld(u, v, fc, cc, R, T, d, h);
rI = worldToImage(rW, fc, cc, R, T, h);
plot(rI(1), rI(2), 'bs')