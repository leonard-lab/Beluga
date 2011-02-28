% compute undistorted location from distorted location
% WARNING: BE AWARE OF PIXEL INDEING STARTING AT 1 OR 0

function [x,y] = undst_point(xp,yp,px,py,px2,py2,good_points)

% (xp,yp) are the point coordinates in the distorted frame found by image
% tracking/feature recognition
% px2 and py2 are the locations of pixels after distortion bounded by the
% image size, px and py are still full set of the undistorted pixel
% location, px(good_points) or ind_new can be used to extract corresponding
% undistorted pixel locations

% 1. search (px2,py2) for points within a set shift for (xp,yp)
s = 1.5;   

close_points = find((abs(px2-xp)<=s)&(abs(py2-yp)<=s));

if isempty(close_points)
%     fprintf(1,'There are no valid pixels in the region of the given point.\n');
    x = 0; y = 0;   % set default/error values
    return;
end

dx = px2(close_points) - xp;
dy = py2(close_points) - yp;
r  = dx.^2 + dy.^2;

% 2. assign weights to these points based on distance from (xp,yp)
w = 10.^(sqrt(2)*s - r) / sum(10.^(sqrt(2)*s - r));

% 3. match each distorted point to its undistorted coordinate point
close_x = px(good_points(close_points));
close_y = py(good_points(close_points));

% 4. compute the location of the undistorted point corresponding to (xp,yp)

x = w*close_x - 1;
y = w*close_y - 1;

end