clear all;  close all;

I = imread('BGD.bmp');

% threshold for how dark the tank is vs. the rim
thresh = 1/3;

% threshold the hsv V channel for how "dark" the image is
H = rgb2hsv(I);
M = (H(:,:,3) < thresh);

% use a median filter to remove salt-and-pepper noise
M = medfilt2(M, [5 5]);

% find connected components in the thresholded image
%  - i.e. dark regions of the image
L = bwlabel(M);

% select the parts of the image corresponding to the largest region
L_most = mode(reshape(L, [numel(L) 1]));
MM = (L == L_most);

% fill in holes in the region to obtain the mask
TankMask = imfill(MM, 'holes');

save('TankMask.mat', 'TankMask');

% show the masked tank
imagesc(immultiply(H(:,:,3), TankMask))