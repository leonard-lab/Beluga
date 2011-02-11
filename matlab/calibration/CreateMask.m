clear all;  close all;

I = imread('BGD.bmp');
H = rgb2hsv(I);

ASK_THRESH = 1;

% threshold for how dark the tank is vs. the rim
thresh = 1/3;
if ASK_THRESH,
    disp('Draw a box including mostly tank pixels');
    
    % waits for the user to draw a rectangle on the image
    imshow(I)
    title('Draw a box including mostly tank pixels')
    waitforbuttonpress
    init_pt = get(gcf, 'CurrentPoint');
    drawn_rect = rbbox([init_pt(1,1) init_pt(1,2) 0 0]);
    
    G = H([drawn_rect(2) : drawn_rect(4)], [drawn_rect(1) : drawn_rect(3)], 3);
    thresh = mean(G(:)) + std(G(:));
end 

% threshold the hsv V channel for how "dark" the image is

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

% erode the image a little - eliminates edge hilights
TankMask = imerode(TankMask, strel('disk', 3));

% remove small extra holes on the side by again finding regions
CC = bwconncomp(~TankMask);
CS = zeros(CC.NumObjects, 1);
for cx = 1 : CC.NumObjects,
    CS(cx) = length(CC.PixelIdxList{cx});
end
[~, L_most] = max(CS);
for cx = 1 : CC.NumObjects,
    % set pixels of all but the largest region to 1
    if cx ~= L_most,
        TankMask(CC.PixelIdxList{cx}) = 1;
    end
end

save('TankMask.mat', 'TankMask');

% show the masked tank
imshow(immultiply(H(:,:,3), TankMask))