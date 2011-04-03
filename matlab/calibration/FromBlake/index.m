function [] = index()

% Extract information from initial image and create template rectified image
load values
k = kc; c = cc; f = fc; alpha = alpha_c;

KK = [f(1) alpha*f(1) c(1);0 f(2) c(2) ; 0 0 1];
save('values','KK','-append')

nr = 480;   nc = 640;   R = eye(3);

buf = 100; % set buffer boarder of pixels

[mx,my] = meshgrid(1-buf:nc+buf, 1-buf:nr+buf);     % mx rows 1:nc, my columns 1:nr, both are nc x nr
px = reshape(mx',(nc+2*buf)*(nr+2*buf),1);          %  [1; 2; 3;...nc ; 1; 2; 3;... ]
py = reshape(my',(nc+2*buf)*(nr+2*buf),1);          %  [1; 1; 1;... ; 2; 2; 2;... nr]

% creates an array of pixel locations, and adds a row of 1s for the
% inverse from pixel coordinates to normalized coortinate points (KK)
rays = inv(KK)*[(px - 1)';(py - 1)';ones(1,length(px))];

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
good_points = find((px_0 >= 0) & (px_0 <= (nc-2)) & (py_0 >= 0) & (py_0 <= (nr-2))...
    &(px' >= 1) & (px' <= (nc-1)) & (py' >= 1) & (py' <= (nr-1)));

pxd = px2;                  pyd = py2;
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

save('values','px*','py*','ind*','a*','nc','nr','buf','-append')