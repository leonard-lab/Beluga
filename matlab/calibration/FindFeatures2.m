clear all;  close all;


% if this is 1, the user supplies the file via the GUI, otherwise a value
% is set manually in the code.  should be 1 except for debugging.
ASK_FOR_FILE = 0;

if ASK_FOR_FILE,
    image_file = uigetfile(...
        {'*.jpg;*.png;*.bmp;*.tif', 'Image Files'; '*.*', 'All Files'}, ...
        'Select an input image', pwd);
    if sum(image_file) == 0 || exist(image_file, 'file') ~= 2,
        error('No valid image file selected.')
    end
else
    image_file = 'Q4.bmp';
end

[~, image_file_basename, ~] = fileparts(image_file);
mask_file = sprintf('TankMask-%s.mat', image_file_basename);

% parameters - change to tweak performance
filter_opts.smoothing_radius = 3;
filter_opts.averaging_window = 5;

center_opts.thresh = 0.8;

seam_opts.search_size = 50;  % 2*n + 1 window around center
seam_opts.search_radius = 45;
seam_opts.search_annulus = 5;
seam_opts.max_iter = 10;

outer_opts.num_peaks = 10;
outer_opts.thresh_factor = 0.1;
outer_opts.gap = 50;
outer_opts.min_length = 100;

side_opts.num_peaks = 50;
side_opts.thresh_factor = 0.01;
side_opts.gap = 5;
side_opts.min_length = 40;
side_opts.contrast_thresh = 0.015;
side_opts.edge_padding = 75;

% set up - configuration (shouldn't need to be changed)
VarName_TankMask = 'TankMask';

% debugging parameters
SKIP_UI = 0;
if SKIP_UI,
    % if SKIP_UI is true, these must be set to reasonable values
    
    % these numbers work with BGD.bmp
    %center_guess = [76 235];
    %edge_guess = [278 480];
    
    % for C1.bmp
    center_guess = [413 411];
    edge_guess = [113 301];
    
    % for C2.bmp
    %center_guess = [70   421];
    %edge_guess = [363   283];
    
    
end
SHOW_CENTER_LINES = 1;

%% read the mask file (with error checking)
if ~exist(mask_file, 'file'),
    error('Could not load mask file %s', mask_file);
end
[~, ~, ext] = fileparts(mask_file);
if ~strcmp(ext, '.mat'),
    error('Mask file %s does not have a .mat extension', mask_file);
end
clear ext

load(mask_file)
if ~exist(VarName_TankMask, 'var')
    error('Mask file %s did not include a variable named ''%s''', ...
        mask_file, VarName_TankMask);
end

%% read the image file (with error checking)
if ~exist(image_file, 'file'),
    error('Image file %s does not exist', image_file);
end

I = imread(image_file);
[frame_height, frame_width, frame_channels] = size(I);

if frame_channels ~= 3,
    %error('Image file %s must be a color image', image_file);
    I = cat(3, I, I, I);
end

figure(1)
imshow(I)
title('Input image')

%% image processing
% color transformations, smoothing, etc

HSV = rgb2hsv(I);
H = HSV(:,:,1); % Hue channel
S = HSV(:,:,2); % Saturation channel
V = HSV(:,:,3); % Value channel (i.e. grayscale image)

% masked grayscale image - what we'll work with most
MV = immultiply(V, TankMask);

% smoothing filter
h_smooth = fspecial('gaussian', filter_opts.smoothing_radius);
% averaaging filter
h_avg = fspecial('average', filter_opts.averaging_window);
% sharpening filter (see doc fspecial for why it's called unsharp)
h_unsharp = fspecial('unsharp');

% apply filters
MV_sharp = immultiply(imfilter(V, h_unsharp), TankMask);
MV_smooth = immultiply(imfilter(V, h_smooth), TankMask);
MV_avg = immultiply(imfilter(V, h_avg), TankMask);

% local contrast map - where the image is brighter or darker than the local
% neighborhood
LocalContrast = double(MV) - double(MV_avg);

imshow(MV)
title('Masked Grayscale Input')

%% user input

% should only be true for debugging - see above
if ~SKIP_UI,

    % get a guess for the center of the tank
%     s = 'Click near the center of the tank';
%     fprintf('%s\n', s)
%     title(s)
%     msgbox(s, 'CreateMode', 'modal')
%     clear s
%     center_guess = fliplr(ginput(1));
%     
%     % get a guess for the edge of the bottom of the tank
%     s = 'Click near the edge of the bottom of the tank';
%     fprintf('%s\n', s)
%     title(s)
%     msgbox(s, 'CreateMode', 'modal')
%     clear s
%     edge_guess = fliplr(ginput(1));
%     
    xmin = 0;
    xmax = 0;
    ymin = 0;
    ymax = 0;
    
    while(abs(xmax - xmin) <= 1 || abs(ymax - ymin) <= 1 || ... 
            xmin < 1 || ymin < 1 || xmax > frame_width || ymax > frame_height),
        
        s = 'Draw a box around the center of the tank';
        fprintf('%s\n',s)
        
        % waits for the user to draw a rectangle on the image
        imshow(I)
        title(s)
        msgbox(s, 'CreateMode', 'modal');
        waitforbuttonpress
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
    end
    
    % get a guess for the edge of the bottom of the tank
    s = 'Click near the edge of the bottom of the tank';
    fprintf('%s\n', s)
    title(s)
    msgbox(s, 'CreateMode', 'modal')
    clear s
    edge_guess = fliplr(ginput(1));
    
end

%% Center Detection

CM = zeros(size(MV));
CM([ymin : ymax], [xmin : xmax]) = 1;
CM = (MV.*CM > center_opts.thresh);
IX = find(CM);
[IX IY] = ind2sub(size(MV),IX);
[z, r] = fitcircle([IY IX]');

TANK_CENTER = z;
TANK_CENTER_RADIUS = r;

figure(1)
imshow(MV)
hold on
plot(TANK_CENTER(1), TANK_CENTER(2), 'rs')
plot(TANK_CENTER(1) + i*TANK_CENTER(2) + r*exp(i*linspace(0,2*pi)), 'b-')
hold off

%% Quadrant mapping: 
%  We can figure out which camera this is by looking at where the
%  center of the tank is in the frame.  The mapping is a reflection about
%  the origin.  Let the quadrant of the center of the tank in the image and
%  relative to the center of the image be given by (i, ii, iii, iv) and let
%  the quadrant of the camera relative to the center of the tank be given
%  by (I, II, III, IV).  The mapping is then, e.g. i -> IV (reflection
%  about the origin).  We can figure out the first part by computing the
%  sign of the coordinates of TANK_CENTER relative to the center of the
%  image.
%
%  Note that the image coordinates are from the top left with ++ being down
%  and right.  We rectify this by negating the y component of Q.
%
Q = TANK_CENTER - [frame_width/2; frame_height/2];
Q = sign(Q);
Q(2) = -Q(2);
if Q(1) >= 0 && Q(2) >= 0,
    % i -> III
    QUADRANT = 3;
elseif Q(1) < 0 && Q(2) >= 0,
    % ii -> IV
    QUADRANT = 4;
elseif Q(1) < 0 && Q(2) < 0,
    % iii -> I
    QUADRANT = 1;
elseif Q(1) >= 0 && Q(2) < 0,
    % iv -> II
    QUADRANT = 2;
end
fprintf('This is the camera in quadrant %d\n', QUADRANT);

%% Edge radius

IX = find(TankMask == 0);
[y x] = ind2sub(size(TankMask), IX);
EdgeInnerRadius = min( sqrt((x - TANK_CENTER(1)).^2 + (y - TANK_CENTER(2)).^2));
hold on
plot(TANK_CENTER(1) + i*TANK_CENTER(2) + EdgeInnerRadius*exp(i*linspace(0,2*pi)), 'g--');
hold off

%% Outer Edge Line Detection

OE_E = edge(TankMask, 'canny');
[OE_Mag, OE_Theta, OE_Radius] = hough(OE_E);
OE_Peaks = houghpeaks(OE_Mag, outer_opts.num_peaks,...
    'threshold', ceil(outer_opts.thresh_factor*max(OE_Mag(:))));
OE_Lines = houghlines(OE_E, OE_Theta, OE_Radius, OE_Peaks,...
    'FillGap', outer_opts.gap, 'MinLength', outer_opts.min_length);

clear OE_Mag OE_Theta OE_Radius OE_Peaks OE_E

M = [];
tx = TANK_CENTER(1);
ty = TANK_CENTER(2);

for ix = 1 : length(OE_Lines),
    r1 = OE_Lines(ix).point1;
    r2 = OE_Lines(ix).point2;
    m = 0.5*(r1 + r2);
    
    M = [M; atan2(m(2)-ty, m(1)-tx) r1 r2 ix];
    
    if SHOW_CENTER_LINES,
        hold on
        plot([r1(1) r2(1)], [r1(2) r2(2)], 'b-x',...
            'MarkerEdgeColor', 'r');
        hold off
    end
end

M = sortrows(M);
M(1, :) = extend_lines(M(1,:), frame_width, frame_height);
for ix = 1 : length(OE_Lines) - 1,
    M([ix ix+1], :) = extend_lines(M([ix ix+1],:), frame_width, frame_height);
end
M(end,:) = extend_lines(M(end,:), frame_width, frame_height);
M = [M sqrt(sum( (M(:, [2 3]) - M(:, [4 5])).^2, 2))];

hold on
for ix = 1 : length(OE_Lines),
    plot(M(ix, [2 4]), M(ix, [3 5]), 'g-o');
end
hold off

clear tx ty r1 r2 OE_Lines m ix d

%% Edge Seam Finding

[nm, ~] = size(M);

dm = 10;
MD = [];
for ix = 1 : nm,
    m = 0.5*(M(ix, [2 3]) + M(ix, [4 5]));
    mx = m(1);  my = m(2);
    
    if (mx > dm) && (frame_width - mx > dm) && (my > dm) && (frame_height - my > dm),
        MD = [MD; [mx my]];
        hold on
        plot(mx, my, 'go')
        plot([TANK_CENTER(1) mx], [TANK_CENTER(2) my], 'g:')
        hold off
    end
end

r_edge_guess = norm(TANK_CENTER' - fliplr(edge_guess));

hold on
plot(TANK_CENTER(1) + i*TANK_CENTER(2) + r_edge_guess*exp(i*linspace(0, 2*pi)), 'b:')
hold off

X = repmat([1 : frame_width], [frame_height 1]) - TANK_CENTER(1);
Y = repmat([1 : frame_height]', [1 frame_width]) - TANK_CENTER(2);
D = sqrt(X.^2 + Y.^2);
mask_edge = immultiply((D > 0.95*r_edge_guess), TankMask);
tank_edge = immultiply(mask_edge, (MV - MV_avg) > side_opts.contrast_thresh);

SS_E = edge(tank_edge, 'canny');
[SS_Mag, SS_Theta, SS_Radius] = hough(SS_E);
SS_Peaks = houghpeaks(SS_Mag, side_opts.num_peaks,...
    'threshold', ceil(side_opts.thresh_factor*max(SS_Mag(:))));
SS_Lines = houghlines(SS_E, SS_Theta, SS_Radius, SS_Peaks,...
    'FillGap', side_opts.gap, 'MinLength', side_opts.min_length);

%clear SS_Mag SS_Theta SS_Radius SS_Peaks

MS = [];
tx = TANK_CENTER(1);  ty = TANK_CENTER(2);
mp = 0.5*(M(:,[2 3]) + M(:,[4 5]));
mean_side_length = mean(M([2 : end-1], 7));
[nm, ~] = size(M);
b = side_opts.edge_padding;

E = [];

for ix = 1 : length(SS_Lines),
    r1 = SS_Lines(ix).point1;
    r2 = SS_Lines(ix).point2;
    m = 0.5*(r1 + r2);
    
    a = atan2(m(2)-ty, m(1)-tx);
    
    d1 = norm([r1 - TANK_CENTER']);
    d2 = norm([r2 - TANK_CENTER']);
    r = abs(d1 - d2);
    if(r < 20)
        continue;
    end
    if (r1(1) < 100 && r2(1) < b) || (r1(1) > frame_width-b && r2(1) > frame_width - b)
        continue;
    end
    if (r1(2) < 100 && r2(2) < b) || (r1(2) > frame_height-b && r2(2) > frame_height-b)
        continue;
    end
    
    d1 = (repmat(M(:, [2 3]), [1 2]) - repmat([r1 r2], [nm 1])).^2;
    d2 = (repmat(M(:, [4 5]), [1 2]) - repmat([r1 r2], [nm 1])).^2;
    d1 = [sqrt(d1(:,1)+d1(:,2)) sqrt(d1(:,3)+d1(:,4))];
    d2 = [sqrt(d2(:,1)+d2(:,2)) sqrt(d2(:,3)+d2(:,4))];
    d1 = min(sqrt((d1 - 0.5*mean_side_length).^2), [], 2);
    d2 = min(sqrt((d2 - 0.5*mean_side_length).^2), [], 2);
    
    
    %d = (repmat(mp, [1 2]) - repmat([r1 r2], [length(mp) 1])).^2;
    %d = [sqrt( d(:, 1) + d(:, 2)) sqrt( d(:, 3) + d(:, 4))];
    
    d = mean([d1 d2], 2);
    [dm, dx] = min(d);
    
    dp1 = M(dx, [2 3]) - M(dx, [4 5]);
    dp2 = r1 - r2;

    MS = [MS; a r1 r2 ix dx dm abs(dp1*dp2')/norm(dp1)/norm(dp2) r];
    
    E = [E; [r1 r2]];
    
    if SHOW_CENTER_LINES,
        hold on
        plot([r1(1) r2(1)], [r1(2) r2(2)], 'y-x',...
            'MarkerEdgeColor', 'r');
        hold off
    end
end

[ne, ~] = size(E);
[nm, ~] = size(MD);
D = zeros(ne, nm);
for ex = 1 : ne,
    r1 = E(ex, [1 2]);
    r2 = E(ex, [3 4]);
    for mx = 1 : nm,
        m = MD(mx, [1 2]);
        d = min( [norm(r1-m) norm(r2-m)]);
        D(ex, mx) = d;
    end
end

[dmin, ix] = min(D(:));
[ex mx] = ind2sub(size(D), ix);

hold on
r1 = E(ex, [1 2]);
r2 = E(ex, [3 4]);
plot([r1(1) r2(1)], [r1(2) r2(2)], 'r-')
hold off

X = repmat([1 : frame_width], [frame_height 1]);% - TANK_CENTER(1);
Y = repmat([1 : frame_height]', [1 frame_width]);% - TANK_CENTER(2);

dx = r2(1)-r1(1);
dy = r2(2)-r1(2);
mx = 0.5*(r1(1) + r2(1));
my = 0.5*(r1(2) + r2(2));
d = sqrt(dx^2 + dy^2);
x1 = r1(1);  y1 = r1(2);

D1 = abs(dx*(y1 - Y) - dy*(x1 - X))/d;
D2 = sqrt( (X - mx).^2 + (Y - my).^2);

mask1 = immultiply((D1 < 20), TankMask);
mask1 = immultiply(mask1, (D2 < d));

L = (mask1.*MV > mean(MV(mask1 > 0)) + 1.5*std(MV(mask1 > 0)));
L = bwlabel(L);
L = L == mode(L(L > 0));
L = find(L > 0);
[Ly Lx] = ind2sub(size(MV), L);

D = sqrt( (Lx - TANK_CENTER(1)).^2 + (Ly - TANK_CENTER(2)).^2 );
[Dmin, ix] = min(D);
BOTTOM_INTERSECTION = [Lx(ix) Ly(ix)];
BOTTOM_RADIUS = Dmin;

hold on
plot(BOTTOM_INTERSECTION(1), BOTTOM_INTERSECTION(2), 'rs');
plot(TANK_CENTER(1) + i*TANK_CENTER(2) + BOTTOM_RADIUS*exp(i*linspace(0, 2*pi)));
hold off

%% Seam Angle Estimation

nr = 6;
Rs = linspace(TANK_CENTER_RADIUS, BOTTOM_RADIUS, nr + 1);
Rs = round(0.5*(Rs([1 : end-1]) + Rs([2 : end])));
%Rs = fliplr(Rs);

S = cell(nr, 1);

g = atan2(BOTTOM_INTERSECTION(2) - TANK_CENTER(2), BOTTOM_INTERSECTION(2) - TANK_CENTER(1));
gs = linspace(0, 2*pi, 15);
gs = gs([1 : end-1]);
g = g + 2*pi/14/2 + gs;
g = atan2(sin(g), cos(g));

tx = TANK_CENTER(1);  ty = TANK_CENTER(2);
for sx = 1 : nr,
    if sx > 1,
        gi = S{sx-1};
        g = [];
        for gx = 1 : length(gi),
            x = tx + Rs(sx)*cos(gi(gx));
            y = ty + Rs(sx)*sin(gi(gx));
            if x > 0 && x < frame_width && y > 0 && y < frame_height,
                g = [g gi(gx)];
            end
        end
    end
    s = FindSeamAnglesAtRadiusFromCenter(MV_smooth, Rs(sx), TANK_CENTER', g);
    hold on
    CS = (Rs(sx)*exp(i*s));
    CS = TANK_CENTER(1) + i*TANK_CENTER(2) + CS;
    plot(CS, 'ro')
    hold off
    S{sx} = s;
end


%% Seam angle sorting
SS = cell(14, 1);
for sx = 1 : length(S{1}),
    p = TANK_CENTER(1) + i*TANK_CENTER(2) + Rs(1)*exp(i*S{1}(sx));
    SS{sx} = [S{1}(sx); real(p); imag(p)];
end

SA = [];
SD = [];
for ax = 1 : 14,
    if isempty(SS{ax}),
        continue;
    end
    a = SS{ax}(1,1);
    for sx = 2 : length(S),
        s = S{sx};
        [md, ix] = min(abs(s - a));
        if md > (2/3)*(2*pi/14/2),
            continue;
        end
        as = s(ix);
        p = TANK_CENTER(1) + i*TANK_CENTER(2) + Rs(sx)*exp(i*as);
        SS{ax} = [SS{ax} [as; real(p); imag(p)]];
    end
    SA = [SA; mean(SS{ax}(1,:))];
    SD = [SD; std(SS{ax}(1,:))];
end

[~,ix_min] = min(abs(SA));
PLUS_X_ANGLE = SA(ix_min);
PLUS_Y_ANGLE = PLUS_X_ANGLE - pi/2;

SEAM_ANGLES = [];
SEAM_ANGLES_W = [];

for ax = 1 : 14,
    
    [~, na] = size(SS{ax});
    if na <= 1 || SD(ax) > 1*pi/180,
        continue;
    end
    
    SEAM_ANGLES = [SEAM_ANGLES; SA(ax)];
    
    k = round( (SA(ax) - PLUS_X_ANGLE)/(2*pi/14) );
    SEAM_ANGLES_W = [SEAM_ANGLES_W; k*2*pi/14];
end

%% output data
% _I -> image (in pixels),  _W -> world (in meters)

% tank center = origin
KNOWN_POINTS_I = [TANK_CENTER'];
KNOWN_POINTS_W = [0 0 0];

tx = TANK_CENTER(1);  ty = TANK_CENTER(2); 
r1_I = TANK_CENTER_RADIUS; 
r2_I = BOTTOM_RADIUS;

r1_W = 0.25;   % radius of center piece at bottom (this is a guess!)
r2_W = 3.048;  % radius of tank (est, based on 20')

% radial points at the inner and outer radius of the seams
for sx = 1 : length(SEAM_ANGLES),
    p_I = tx + i*ty + r2_I*exp(i*SEAM_ANGLES(sx));
    if ~(real(p_I) < 1 || imag(p_I) < 1 || real(p_I) > frame_width || imag(p_I) > frame_height),
        p_W = r2_W*exp(i*SEAM_ANGLES_W(sx));
        KNOWN_POINTS_I = [KNOWN_POINTS_I; real(p_I) imag(p_I)];
        KNOWN_POINTS_W = [KNOWN_POINTS_W; real(p_W) -imag(p_W) 0];
    end
    
    p_I = tx + i*ty + 0.5*(r1_I+r2_I)*exp(i*SEAM_ANGLES(sx));
    if ~(real(p_I) < 1 || imag(p_I) < 1 || real(p_I) > frame_width || imag(p_I) > frame_height),
        p_W = 0.5*(r1_W+r2_W)*exp(i*SEAM_ANGLES_W(sx));
        KNOWN_POINTS_I = [KNOWN_POINTS_I; real(p_I) imag(p_I)];
        KNOWN_POINTS_W = [KNOWN_POINTS_W; real(p_W) -imag(p_W) 0];
    end
    
end

r3_W = r2_W - .05;  % radius to inside of corners at the top (guess)
z3_W = 2.438;       % height to top of tank edge from bottom of inside of tank (est, based on 8')

% corners along the top inner edge
[nm, ~] = size(M);

p_I = [M(2, 2) M(2, 3)];
a = atan2(M(2,3) - ty, M(2,2) - tx);
[~, ix_min] = min(abs(SA - a));
k = round( (SA(ix_min) - PLUS_X_ANGLE)/(2*pi/14) );
p_W = r3_W*exp(i*k*2*pi/14);
KNOWN_POINTS_I = [KNOWN_POINTS_I; p_I];
KNOWN_POINTS_W = [KNOWN_POINTS_W; [real(p_W) -imag(p_W) z3_W]];

p_I = [M(2, 4) M(2, 5)];
a = atan2(M(2,5) - ty, M(2,4) - tx);
[~, ix_min] = min(abs(SA - a));
k = round( (SA(ix_min) - PLUS_X_ANGLE)/(2*pi/14) );
p_W = r3_W*exp(i*k*2*pi/14);
KNOWN_POINTS_I = [KNOWN_POINTS_I; p_I];
KNOWN_POINTS_W = [KNOWN_POINTS_W; [real(p_W) -imag(p_W) z3_W]];

for mx = 3 : nm-1,
    p_I = [M(mx, 4) M(mx, 5)];
    a = atan2(M(mx,5) - ty, M(mx,4) - tx);
    [~, ix_min] = min(abs(SA - a));
    k = round( (SA(ax) - PLUS_X_ANGLE)/(2*pi/14) );
    p_W = r3_W*exp(i*k*2*pi/14);
    KNOWN_POINTS_I = [KNOWN_POINTS_I; p_I];
    KNOWN_POINTS_W = [KNOWN_POINTS_W; [real(p_W) -imag(p_W) z3_W]];
end

hold on
plot(KNOWN_POINTS_I(:,1), KNOWN_POINTS_I(:,2), 'bs', 'MarkerSize', 5)
hold off
    

figure(2)
clf
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
