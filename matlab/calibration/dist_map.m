% DISTORTION MAPPING FOR FORRESTAL CAMERAS
%
% In this program we take a single input image and, using the intrinsic
% camera properties from one set of checkerboard callibration, compute the
% transformation between distorted and real pixel locations
%
% INPUTS: image (used for size and mapping check) and intrinsic camera
% properties - focal length, camera center, distortion coefficients, skew

function [Xrect,Yrect] = dist_map(I,f,c,k,alpha)

% compute the KK matrix transform to pixel coordinates
KK = [f(1) alpha*f(1) c(1);0 f(2) c(2) ; 0 0 1];

% SHOW THE ORIGINAL IMAGE: 
quest = input('Do you want to display distorted image? (yes/no) ','s');
if isempty(quest),    quest = 'no';     end;

if strcmp(quest,'yes'),
    figure(1);    image(I);     colormap(gray(256));
    title('Original image (with distortion) - Stored in array I');
    drawnow;
end

% UNDISTORT THE IMAGE:
fprintf(1,'Computing the undistorted image...\n')

[I2,Xrect,Yrect] = rct(I,eye(3),f,c,k,alpha,KK);

fprintf(1,'done\n');

% SHOW THE UNDISTORTED IMAGE:
quest = input('Do you want to display corrected image? (yes/no) ','s');
if isempty(quest),    quest = 'no';     end;

if strcmp(quest,'yes'),
    figure(2);  image(I2);  colormap(gray(256));    
    title('Undistorted image - Stored in array I2');
    drawnow;
end

end

%  may choose to add additional features,or input commands