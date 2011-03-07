clear all;  close all;


% connected components of the dark region of the mask that are smaller than
% this are discarded
MIN_COMP_SIZE = 1000;

% if this is 1, the user will select a region to automatically determine a
% threshold for the tank.  Otherwise, 1/3 is used.  Should be 1 except for
% debugging purposes
ASK_THRESH = 1;

% if this is 1, the user supplies the file via the GUI, otherwise a value
% is set manually in the code.  should be 1 except for debugging.
ASK_FOR_FILE = 1;

if ASK_FOR_FILE,
    image_file = uigetfile(...
        {'*.jpg;*.png;*.bmp;*.tif', 'Image Files'; '*.*', 'All Files'}, ...
        'Select an input image', pwd);
    if sum(image_file) == 0 || exist(image_file, 'file') ~= 2,
        error('No valid image file selected.')
    end
else
    image_file = 'Q3.bmp';
end    

I = imread(image_file);
[frame_height, frame_width, nchannels] = size(I);
if nchannels == 3,
    H = rgb2hsv(I);
    V = H(:,:,3);
else
    H = I;
    V = double(I)/255;
end

% threshold for how dark the tank is vs. the rim
thresh = 1/3;
if ASK_THRESH,
    disp('Draw a box including mostly tank pixels');
    
    % waits for the user to draw a rectangle on the image
    imshow(I)
    title('Draw a box including mostly tank pixels')
    waitforbuttonpress
    init_pt = get(gcf, 'CurrentPoint');
    p1 = get(gca, 'CurrentPoint');
    drawn_rect = rbbox([init_pt(1,1) init_pt(1, 2) 0 0]);
    p2 = get(gca, 'CurrentPoint');
    
    p1 = p1(1, [1 2]);
    p2 = p2(1, [1 2]);
    xmin = floor(min([p1(1) p2(1)]));
    xmax = floor(max([p1(1) p2(1)]));
    ymin = floor(min([p1(2) p2(2)]));
    ymax = floor(max([p1(2) p2(2)]));
    
    G = V([ymin : ymax], [xmin : xmax]);
    thresh = mean(G(:)) + 1.5*std(G(:));
end 

% threshold the hsv V channel for how "dark" the image is

M = (V < thresh);

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
T_a = TankMask;

% erode the image a little - eliminates edge hilights
TankMask = imerode(TankMask, strel('disk', 3));
T_b = TankMask;

% remove small extra holes on the side by again finding regions
CC = bwconncomp(~TankMask);
CS = zeros(CC.NumObjects, 1);
CP = zeros(CC.NumObjects, 1);
r = regionprops(CC, 'Area', 'Perimeter', 'PixelList');
for cx = 1 : CC.NumObjects,
    CS(cx) = r(cx).Area;
    CP(cx) = r(cx).Perimeter;
end
for cx = 1 : CC.NumObjects,
    % set pixels of all but the largest region to 1
    if CS(cx) < MIN_COMP_SIZE,
        TankMask(CC.PixelIdxList{cx}) = 1;
    end
    if RegionIntersectsBox(repmat([frame_height 0], [CS(cx) 1]) - r(cx).PixelList, [xmin ymin xmax ymax]),
        TankMask(CC.PixelIdxList{cx}) = 1;
    end
end

[~, basename, ~] = fileparts(image_file);
save(sprintf('TankMask-%s.mat', basename), 'TankMask');

% show the masked tank
imshow(immultiply(V, TankMask))