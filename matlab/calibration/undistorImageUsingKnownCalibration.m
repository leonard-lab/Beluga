clear all;  close all;

[fc, cc, alpha_c, kc, ~, ~, ~] = ReadCalibrationData('SnellCalib-Q1.dat');
KK = [fc(1) alpha_c*fc(1) cc(1);0 fc(2) cc(2) ; 0 0 1];

addpath 'toolbox_calib'

ASK_FOR_FILE = 1;

if ASK_FOR_FILE,
    [image_file, image_path] = uigetfile(...
        {'*.jpg;*.png;*.bmp;*.tif', 'Image Files'; '*.*', 'All Files'}, ...
        'Select an input image', pwd);
    image_file = fullfile(image_path, image_file);
    if sum(image_file) == 0 || exist(image_file, 'file') ~= 2,
        error('No valid image file selected.')
    end
else
    image_file = 'Q3.bmp';
end    

I = imread(image_file);

[Ipart_1] = rect(I(:,:,1),eye(3),fc,cc,kc,alpha_c,KK);
[Ipart_2] = rect(I(:,:,2),eye(3),fc,cc,kc,alpha_c,KK);
[Ipart_3] = rect(I(:,:,3),eye(3),fc,cc,kc,alpha_c,KK);

I2 = ones(ny, nx,3);
I2(:,:,1) = Ipart_1;
I2(:,:,2) = Ipart_2;
I2(:,:,3) = Ipart_3;

imshow(I2)

rmpath 'toolbox_calib'