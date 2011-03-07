clear all;  close all;

% set up - input file names
image_file = 'C1.bmp';

[~, image_file_basename, ~] = fileparts(image_file);
mask_file = sprintf('TankMask-%s.mat', image_file_basename);

% parameters - change to tweak performance
filter_opts.smoothing_radius = 3;
filter_opts.averaging_window = 5;

center_opts.num_peaks = 40;
center_opts.thresh_factor = 0.1;
center_opts.gap = 10;
center_opts.min_length = 50;
center_opts.max_dist = 40;

seam_opts.search_size = 50;  % 2*n + 1 window around center
seam_opts.search_radius = 45;
seam_opts.search_annulus = 5;

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
SKIP_UI = 1;
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
    error('Image file %s must be a color image', image_file);
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
    s = 'Click near the center of the tank';
    fprintf('%s\n', s)
    title(s)
    msgbox(s, 'CreateMode', 'modal')
    clear s
    center_guess = fliplr(ginput(1));
    
    % get a guess for the edge of the bottom of the tank
    s = 'Click near the edge of the bottom of the tank';
    fprintf('%s\n', s)
    title(s)
    msgbox(s, 'CreateMode', 'modal')
    clear s
    edge_guess = fliplr(ginput(1));

end

%% Center Detection: Line Detection

CH_E = edge(MV_smooth, 'canny');
[CH_Mag, CH_Theta, CH_Radius] = hough(CH_E);
CH_Peaks = houghpeaks(CH_Mag, center_opts.num_peaks,...
    'threshold', ceil(center_opts.thresh_factor*max(CH_Mag(:))));
CH_Lines = houghlines(CH_E, CH_Theta, CH_Radius, CH_Peaks,...
    'FillGap', center_opts.gap, 'MinLength', center_opts.min_length);

clear CH_Mag CH_Theta CH_Radius CH_Peaks
% CH_E is used below

%% Center Detection:  Line Filtering & Center Estimation

lines_in = CH_Lines;
CH_Lines = [];

C = flipud(repmat(center_guess', [1 2]));
Phi = [];  Y = [];
for ix = 1 : length(lines_in),
    r1 = lines_in(ix).point1';
    r2 = lines_in(ix).point2';

    % distance from line endpoints to tank center
    d = min(sqrt(sum( ([r1 r2] - C).^2 )));
    % distance from line midpoint to tank center
    c = sqrt(sum((0.5*(r1 + r2) - C(:,1)).^2));
    
    % ignore lines too far from tank center and lines whose midpoint is
    % closer to the center than either endpoint
    if (d > center_opts.max_dist) || (c < d),
        continue;
    end
    
    CH_Lines = [CH_Lines; lines_in(ix)];
    
    dx = r2(1) - r1(1);
    dy = r2(2) - r1(2);
    dr = sqrt(dx^2 + dy^2);
    Phi = [Phi; dy/dr -dx/dr];
    Y = [Y; -(1/dr)*(dx*r1(2) - dy*r1(1))];
    
    if SHOW_CENTER_LINES,
        hold on
        plot([r1(1) r2(1)], [r1(2) r2(2)], 'b-x',...
            'MarkerEdgeColor', 'r');
        hold off
    end
end

TANK_CENTER = Phi\Y;

hold on
plot(center_guess(2), center_guess(1), 'mo')
plot(TANK_CENTER(1), TANK_CENTER(2), 'gs')
hold off

% Quadrant mapping: 
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
    QUADRANT = 'III';
elseif Q(1) < 0 && Q(2) >= 0,
    % ii -> IV
    QUADRANT = 'IV';
elseif Q(1) < 0 && Q(2) < 0,
    % iii -> I
    QUADRANT = 'I';
elseif Q(1) >= 0 && Q(2) < 0,
    % iv -> II
    QUADRANT = 'II';
end
fprintf('This is the camera in quadrant %s\n', QUADRANT);

clear lines_in Phi Y dx dy dr d c r1 r2 C ix

%% Centerpiece Radius Estimation & Seam Angles

CR_R = [];
S_TH = [];
S_V = [];

tx = TANK_CENTER(1);
ty = TANK_CENTER(2);

for x = round(tx) - seam_opts.search_size : ...
        round(tx) + seam_opts.search_size,
    for y = round(ty) - seam_opts.search_size : ...
            round(ty) + seam_opts.search_size,
        
        r = sqrt((tx-x)^2 + (ty-y)^2);
        if CH_E(y, x),
            CR_R = [CR_R; r];
        end
        
        if abs(round(r) - seam_opts.search_radius) < seam_opts.search_annulus,
            S_TH = [S_TH; atan2(x - tx, y - ty)];
            S_V = [S_V; MV_avg(y, x)];
        end
    end
end

[n, b] = hist(CR_R, [min(CR_R) : 1 : max(CR_R)]);
n = n(2 : end-1);
b = b(2 : end-1);
[~, mx] = sort(n, 2, 'descend');
TANK_CENTER_RADIUS = mean(b(mx([1 : 3])));

hold on
plot(TANK_CENTER(1) + i*TANK_CENTER(2) + TANK_CENTER_RADIUS*exp(i*linspace(0, 2*pi)), 'g-');
hold off

% detrending S_V - find best-fit sinusoidal trend
%  - a sinusoidal trend is consistent with the illumination changing
%    roughly linearly across the tank
[S_TH, ix] = sort(S_TH);
S_V = S_V(ix);

S_V = smooth(S_V, 2*floor(length(S_V)/14/5/2) + 1);

a0 = mean(S_V);
Phi = [sin(S_TH) cos(S_TH) sin(2*S_TH) cos(2*S_TH)];
Y = [S_V - a0];
A = Phi\Y;
S_V = S_V - (a0 + A(1)*sin(S_TH) + A(2)*cos(S_TH) ...
    + A(3)*sin(2*S_TH) + A(4)*cos(2*S_TH));
% remove any remaining linear trend
S_V = detrend(S_V);


%S_b = S_V < (mean(S_V) - (1/3)*(mean(S_V) - min(S_V)));
%S_b = S_V < (mean(S_V) - 0.5*std(S_V));
S_b = S_V < 0;
S_t = sort(S_TH(S_b > 0));

d = [diff(S_t)' 0];
dL = [];
t_d = 0.5;
while length(dL) ~= 13,
    dL = find(d > t_d*max(d));
    if(length(dL) < 14),
        t_d = t_d - 0.1;
    else
        t_d = t_d + 0.1;
    end
end

SEAM_ANGLES = zeros(1,14);
SEAM_ANGLES(1) = mean(S_t(1 : dL(1)));
for ix = 1 : 12,
    SEAM_ANGLES(ix+1) = mean(S_t(dL(ix)+1 : dL(ix + 1)));
end
SEAM_ANGLES(end) = mean(S_t(dL(end) : end));
g = exp(i*SEAM_ANGLES);
SEAM_ANGLES = atan2(real(g), imag(g));

hold on
CS = (seam_opts.search_radius*exp(i*SEAM_ANGLES));
CS = TANK_CENTER(1) + i*TANK_CENTER(2) + CS;
plot(CS, 'ro')
hold off

clear CR_R S_TH S_V tx ty x y r n b mx S_b S_t t_d d dL ix g CS

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

d = sqrt(sum( (repmat(TANK_CENTER', [length(OE_Lines) 1])...
    - M(:,[4 5])).^2, 2));
TANK_TOP_RADIUS = mean(d);

hold on
plot(TANK_CENTER(1) + i*TANK_CENTER(2) + TANK_TOP_RADIUS*exp(i*linspace(0, 2*pi)), 'g:')
hold off

clear tx ty r1 r2 OE_Lines m ix d

%% Side seam edge finding

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
    
    if SHOW_CENTER_LINES,
        hold on
        plot([r1(1) r2(1)], [r1(2) r2(2)], 'y-x',...
            'MarkerEdgeColor', 'r');
        hold off
    end
end
