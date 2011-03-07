clear all;
close all;

N = 400;
I = zeros(N);

Q = 1;
L = [1 : floor(N/2)];
R = [floor(N/2)+1 : N];
C = [floor(N/3) : 2*floor(N/3)];
switch(Q),
    case 1,
        I(R, R) = 1;
    case 2,
        I(R, L) = 1;
    case 3,
        I(L, R) = 1;
    case 4,
        I(L, L) = 1;
    case 5,
        I(C, C) = 1;
end

imshow(I)

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
    
CC = bwconncomp(I);

p = regionprops(CC, 'PixelList');
 
b = RegionIntersectsBox(p.PixelList, [xmin ymin xmax ymax])

I([ymin : ymax], [xmin : xmax]) = 1;
imshow(I)