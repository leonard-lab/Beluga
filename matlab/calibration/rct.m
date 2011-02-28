function [Irec,Xrect,Yrect] = rct(I,R,f,c,k,alpha,KK_new)

% Extract information from initial image and create template rectified image
[nr,nc] = size(I);
Irec = 255*ones(nr,nc);

[mx,my] = meshgrid(1:nc, 1:nr);     % mx rows 1:nc, my columns 1:nr, both are nc x nr
px = reshape(mx',nc*nr,1);          %  [1; 2; 3;... 1; 2; 3;... ]
py = reshape(my',nc*nr,1);          %  [1; 1; 1;... 2; 2; 2;... ]

% creates an array of pixel locations, and adds a row of 1s for the
% inverse from pixel coordinates to normalized coortinate points (KK)
rays = inv(KK_new)*[(px - 1)';(py - 1)';ones(1,length(px))];

% Rotation (or affine transformation) and normalization by third column:
rays2 = R'*rays;
x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)]; % points normalized by third vector component

% Add distortion: this computes a matrix xd of the points of the distorted 
% set of points corresponding to the undistorted coordinates x (not pixel locations)
xd = distort(x,k);

% Reconvert in pixels (essentially by applying KK matrix):
px2 = f(1)*(xd(1,:)+alpha*xd(2,:))+c(1);
py2 = f(2)*xd(2,:)+c(2);

% Interpolate between the closest pixels:
px_0 = floor(px2);
py_0 = floor(py2);

% Select the points that, when distorted, still fall within the bounds of
% the image, these are the points which we have in the distorted image
good_points = find((px_0 >= 0) & (px_0 <= (nc-2)) & (py_0 >= 0) & (py_0 <= (nr-2)));

px2 = px2(good_points);     py2 = py2(good_points);   % subset of the pixels
px_0 = px_0(good_points);   py_0 = py_0(good_points); % subset of pixel floor

% Compute difference between distorted coordinate and discrete pixel location
alpha_x = px2 - px_0; % distance from location to discrete pixel left
alpha_y = py2 - py_0; % distance from location to discrete pixel above

% Create weightings for 
a1 = (1 - alpha_y).*(1 - alpha_x);  % distance right and down weights pixel left and up
a2 = (1 - alpha_y).*alpha_x;        % weight pixel right and up
a3 = alpha_y .* (1 - alpha_x);      % weight pixel left and down
a4 = alpha_y .* alpha_x;            % weight pixel right and down

ind_lu = px_0 * nr + py_0 + 1;      % index for the left upper pixel
ind_ru = (px_0 + 1) * nr + py_0 + 1;
ind_ld = px_0 * nr + (py_0 + 1) + 1;
ind_rd = (px_0 + 1) * nr + (py_0 + 1) + 1;

% create a subset of points that correspond to the pixels in the rectified
% image for which their distortion is in the original image and can be
% computed from this
ind_new = (px(good_points)-1)*nr + py(good_points);

% fill in the correct image (Irec) by taking the weighted average of the 4
% pixels in the region of the original image, that the undistorted pixel is
% distorted to in the mapping from px >> px2
Irec(ind_new) = a1 .* I(ind_lu) + a2 .* I(ind_ru) + a3 .* I(ind_ld) + a4 .* I(ind_rd);

% fprintf('got here!\n')

% CHECK SECTION:
% check_works = find((abs(px2-200)<=1.5)&(abs(py2-200)<=1.5))

% BUILD XRECT AND YRECT
Xrect = zeros(nr,nc);
Yrect = zeros(nr,nc);

bad_pix = 0;

while true
fprintf(1,['\n Input a point to be undistorted (' num2str(min(px2)) ' < x < ' ...
    num2str(max(px2)) ', and ' num2str(min(py2)) ' < y < ' num2str(max(py2)) ')\n'])

xsearch = input('x = ');
ysearch = input('y = ');

for i = xsearch
    for j = ysearch
        [x_temp,y_temp] = undst_point(xsearch,ysearch,px,py,px2,py2,good_points);
        
        Xrect(j+1,i+1) = x_temp;
        Yrect(j+1,i+1) = y_temp;
        
        if ((x_temp == 0) &&(y_temp ==0))
            fprintf('This point could not be computed\n')
            bad_pix = bad_pix + 1;
            disp(['the number of bad pixels is: ' num2str(bad_pix)])
        else
            fprintf(1,['x-distorted = ' num2str(i) ' y-distorted = ' ...
                num2str(j) ' corresponds to, x-true = ' num2str(x_temp)...
                ' y-true = ' num2str(y_temp) ' \n']);

        end
            
    end
end
end
end