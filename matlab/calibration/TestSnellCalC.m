clear all;  close all;

Q = 1;

im_file = sprintf('Q%d.bmp', Q);
im_file_d = sprintf('Q%dd.bmp', Q);
cal_file = sprintf('SnellCalib-Q%d.dat', Q);

out_file = sprintf('tmp_Q%d_o.dat', Q);
in_file = sprintf('tmp_Q%d_i.dat', Q);

%./test_CoordinateTransform 0 ../matlab/calibration/SnellCalib-Q1.dat t1.dat t2.dat no

CTexe = '../../test/test_CoordinateTransform';
I2W = '0';
W2I = '1';

r2_W = 6.547/2;  % radius of tank (est, based on pro-e model)

I = imread(im_file);
[nrows, ncols] = size(I);

Id = imread(im_file_d);

[fc, cc, alpha_c, kc, R, T, lines] = ReadCalibrationData(cal_file);
C = -R'*T;

%cc = [320 240];
%fc = [472 472];

figure(1)
subplot(1,2,1)
imshow(I)
hold on
subplot(1,2,2)
imshow(Id)
hold on

% water depth est 7.5'
w = 2.286;
h = C(3)-w;

rW = [r2_W 0 0]';
rI = worldToImage(rW, fc, cc, R, T, h);

save(out_file, 'rW', '-ascii');
CTcmd = sprintf('! %s %s %s %s %s %s', CTexe, W2I, cal_file, out_file, in_file, 'no');
eval(CTcmd);
rIc = load(in_file, '-ascii');

subplot(1,2,1)
plot(rI(1), nrows - rI(2), 'gs', rIc(1), nrows - rIc(2), 'rs')

CTcmd = sprintf('! %s %s %s %s %s %s', CTexe, W2I, cal_file, out_file, in_file, 'yes');
eval(CTcmd);
rIc = load(in_file, '-ascii');

subplot(1,2,2)
plot(rIc(1), nrows - rIc(2), 'rs')

rd = [rIc(1) rIc(2) w-rW(3)];
save(out_file, 'rd', '-ascii');
CTcmd = sprintf('! %s %s %s %s %s %s', CTexe, I2W, cal_file, out_file, in_file, 'yes');
eval(CTcmd)
rWc = load(in_file, '-ascii');

rd = [rI(1) rI(2) w-rW(3)];
save(out_file, 'rd', '-ascii');
CTcmd = sprintf('! %s %s %s %s %s %s', CTexe, I2W, cal_file, out_file, in_file, 'no');
eval(CTcmd)
rW2c = load(in_file, '-ascii')';

rW2 = imageToWorld(rI(1), rI(2), fc, cc, R, T, w-rW(3), h);

return

subplot(1,2,1)

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