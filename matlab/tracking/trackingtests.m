clear all;
close all;

load '../calibration/TankMask.mat' TankMask
addpath('../calibration')

thresh = 0.18;

I1 = imread('T1.bmp');
H1 = rgb2hsv(I1);
image(I1);

B1 = H1(:,:,3);
T1 = (B1 < thresh).*TankMask;
T1 = medfilt2(T1);

I2 = imread('T3.bmp');
H2 = rgb2hsv(I2);
image(I2);

B2 = H2(:,:,3);
T2 = (B2 < thresh).*TankMask;
T2 = medfilt2(T2);

figure
subplot(2,2,1)
image(I1)
subplot(2,2,2)
image(I2)
subplot(2,2,3)
imagesc(T1);
colormap gray
subplot(2,2,4)
imagesc(T2);
colormap gray
