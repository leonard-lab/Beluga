% Camera Initialization: Coordinate Transforms

clc, clear all, close all
q = input('Please input the quadrants to be calibrated: ');
% if isempty(q),    q = 1:4;     end; % set default all/no quadrants

depth = input('\nPlease input the current depth of water in the tank (m): ');
if isempty(depth),    depth = 2.4384;     end; % set default depth to be 8ft
if depth >= 5 && depth <12, depth = depth*0.3048; end    % convert from ft
if depth >=12,              depth = depth*0.0254; end    % convert from in
save('values','depth','-append')

snl = input('\nDo you want to run calibration with snell''s law? (yes/no) ','s');
if strcmp(snl,'y'), snl = 'yes';    end;
if isempty(snl) || strcmp(snl,'n'),    snl = 'no';     end;


for i = q
    % set the image file name to use here - i.e. Q1, Q2, Q3, Q4
    ImageFile = sprintf('Q%i.bmp',i);

    % loading the data:  the data file name is keyed on the image file name,
    % i.e. CalibPoints-Q1.dat, etc
    [~, image_file_basename, ~] = fileparts(ImageFile);
    datafile = sprintf('CalibPoints-%s.dat', image_file_basename);
    D = load(datafile);

    % the first two columns are the pixel coordinates, the remaining three are
    % world X, Y, and Z
    % _I for image (pixels), _W for world (meters)
    KNOWN_POINTS_I = D(:, [1 2]);
    KNOWN_POINTS_W = D(:, [3 4 5]);
    
    [R,T] = transform(KNOWN_POINTS_I', KNOWN_POINTS_W',depth,snl);
    
    eval(['R_' num2str(i) ' = R;']);
    eval(['T_' num2str(i) ' = T;']);
    
    % display the results for R and T matrices
    fprintf('\n')
    disp(['The rotation matrix (R) for quadrant ' num2str(i) ' is: '])
    disp(eval(['R_' num2str(i)]))

    disp(['The translation vector (Tw) for quadrant ' num2str(i) ' is: '])
%     disp(eval(['T_' num2str(i)])) % in camera frame
    disp(eval(['R_' num2str(i) '''*T_' num2str(i)])) % in world frame

end

% load values
% clear q ImageFile R T D KNOWN* datafile i image_file_basename
if ~isempty(q), save('values','R_*','T_*','depth','-append') % only if computed
disp('Save Complete: values file has been updated'), 
end 

% distortion mapping
quest = input('\n Do you want to recompute indexing? (yes/no): ','s');
if strcmp(quest,'y') || strcmp(quest,'yes')
    index();
end

% undistort images
load Images
while true
quest = input('\n Please input images to be undistorted (must be in Images workspace): ');
if isempty(quest)
    break
end
[~] = Irct(quest);
end
%%
% method of undistorting image points
opt1 = input('\n Do you want to undistort pixels by computation (0) or stored matrices (1)? ');
if isempty(opt1),    opt1 = 0;   end
if opt1 == 1;
    opt = input('\n Do you want to recompute matrices (1)? ');
    if opt == 1;
        [Xrect,Yrect] = rect();
        save('values','*rect','-append')
    end
end

% undistorting image points
opt2 = input('\n Do you want to ignore the effect of snell''s law (1)? ');
if isempty(opt2) || opt2 ~= 1, opt2 = 0; end

while true
    xsearch = input('x = ');
    ysearch = input('y = ');
    
    if isempty(xsearch) && isempty(ysearch)
        break
    end
    
    dep     = input('Vehicle Depth = ');
    quad    = input('Quadrant: ');
       
    [wrld_crds] = realworld([xsearch;ysearch],dep,opt1,opt2,quad);
    x = wrld_crds(1);   y = wrld_crds(2);   z = wrld_crds(3);
    
    fprintf(1,['\nRelative to the world frame, fixed to the bottom, center of the tank, the vehicle is at:'...
         '\nXw = ' num2str(x) '\t Yw = ' num2str(y) '\t Zw = ' num2str(z) ' \n\n']);
end

fprintf('\n Done\n')

%% Test Method for Computing Accuracy of Calibration
% we test the accuracy of the calibration by recomputing the world location
% of the calibration points from their known depths and pixel coordinates,
% as this is the information that we will have on the vehicles from
% tracking software and depth sensor
load values depth

for i = 1:4

    % set the image file name to use here - i.e. Q1, Q2, Q3, Q4
    ImageFile = sprintf('Q%i.bmp',i);

    % loading the data:  the data file name is keyed on the image file name,
    % i.e. CalibPoints-Q1.dat, etc
    [~, image_file_basename, ~] = fileparts(ImageFile);
    datafile = sprintf('CalibPoints-%s.dat', image_file_basename);
    D = load(datafile);

    % the first two columns are the pixel coordinates, the remaining three are
    % world X, Y, and Z
    % _I for image (pixels), _W for world (meters)
    im_pts = D(:, [1 2])';
    wrld_pts = D(:, [3 4 5])';
    
    rad = zeros(1,size(im_pts,2)); % distance from 
    
    disp('The distance of the computed world point from the ideal: ')
    
    for j = 1: size(im_pts,2)
        d = depth - wrld_pts(3,j);
        dst = test_pnt(im_pts(1,j),im_pts(2,j));
        [wrld_crds] = realworld(dst,d,0,0,i); 
        
        dx = wrld_crds(1) - wrld_pts(1,j);
        dy = wrld_crds(2) - wrld_pts(2,j);
        dz = wrld_crds(3) - wrld_pts(3,j);
        rad(j) = norm(wrld_crds - wrld_pts(:,j)); 
%         
%         fprintf(1,['\nRelative to ideal position, the computed point is:'...
%          '\ndX = ' num2str(dx) '\t dY = ' num2str(dy) '\t dZ = ' num2str(dz) ' \n']);
%         fprintf(1,['Point ' num2str(j) ' error = ' num2str(rad(j)) ' m\n'])

    end
                
    average = mean(rad);
    fprintf(1,['Quadrant ' num2str(i) ' Average error = ' num2str(average) ' m\n\n'])
    
end

        
        