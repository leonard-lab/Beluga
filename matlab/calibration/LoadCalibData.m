clear all;  close all;

% set the image file name to use here - i.e. Q1, Q2, Q3, Q4
ImageFile = 'Q4.bmp';

% loading the data:  the data file name is keyed on the image file name,
% i.e. CalibPoints-Q1.dat, etc
[~, image_file_basename, ~] = fileparts(ImageFile);
datafile = sprintf('CalibPoints-%s.dat', image_file_basename);
D = load(datafile);

% the first two columns are the pixel coordinates, the remaining three are
% world X, Y, and Z
% _I for image (pixels), _W for world (meters)
KNOWN_POINTS_I = D(:, [1 2]);
KNOWN_POINTS_W = D(:, [3 4 5]);


% the rest is just for vizualization
I = imread(ImageFile);

wx = KNOWN_POINTS_W(:,1);
wy = KNOWN_POINTS_W(:,2);
wz = KNOWN_POINTS_W(:,3);

z3_W = max(wz);
r2_W = max( sqrt(wx.^2 + wy.^2));

figure(1)
set(gcf, 'Color', 'w', 'Units', 'inches', 'Position', [0.5 0.5 10 5]);

subplot(1,2,1)
imshow(I)
hold on
plot(KNOWN_POINTS_I(:,1), KNOWN_POINTS_I(:,2), 'bs', 'MarkerSize', 5)
subplot(1,2,2)

c1 = r2_W*exp(i*linspace(0, 2*pi));
plot3(KNOWN_POINTS_W(:,1), KNOWN_POINTS_W(:, 2), KNOWN_POINTS_W(:,3), 'bs', ...
    real(c1), imag(c1), zeros(size(c1)), 'k-', ...
    real(c1), imag(c1), z3_W*ones(size(c1)), 'k-');
axis equal

D = [KNOWN_POINTS_I KNOWN_POINTS_W];
save(sprintf('CalibPoints-%s.dat', image_file_basename), 'D', '-ascii');