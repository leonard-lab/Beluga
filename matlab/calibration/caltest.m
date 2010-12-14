clear all;  close all;

I = imread('BGD.bmp');

B = rgb2gray(I);
E = edge(B, 'canny');

figure(1)
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

