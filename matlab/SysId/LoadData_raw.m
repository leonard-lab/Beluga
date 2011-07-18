clear all;  close all; clc;

addpath('d:\dsrc\MADTraC\xdf\matlab\');
addpath('../Calibration/');

%f = ReadXDF(fullfile(getenv('USERPROFILE'), 'Desktop/test.xdf'));
f = ReadXDF('./testdan.xdf');
dataname = 'testdan';

% depth calibration info:
depth_offset = 100;
depth_scale = 0.5;
water_depth = str2num(GetValueFromXDInfoByName(f, 'Water_Depth'));

% how many objects were we tracking?
N = str2num(GetValueFromXDInfoByName(f, 'Number_of_Objects_Tracked'));

% look at f.DataFiles.Name for a list of files available
t = LoadXDFDataFileByName(f, 'Time');
t = t - t(1);
Nt = length(t);

% Raw measurements from each camera, formatted as e.g.:
% x_1_1 x_1_2 x_1_3 x_1_4 x_2_1 x_2_2 ... etc
% where x_i_j is the x position of object i in camera j
% 
% When a measurement is unavailable (i.e. object not in that camera),
% the value is set to -1.  If one variable is -1, they should all be -1,
% with the exception of the orientation, which can take on negative values
rawX = LoadXDFDataFileByName(f, 'Blob_X');
rawY = LoadXDFDataFileByName(f, 'Blob_Y');
rawA = LoadXDFDataFileByName(f, 'Blob_Area');
rawO = LoadXDFDataFileByName(f, 'Blob_Orientation');

% key valid values on the area, which should never be negative for valid
% measurements
Ix = (rawA >= 0);
X_cam = nan(size(rawX));
X_cam(Ix) = rawX(Ix);
Y_cam = nan(size(rawY));
Y_cam(Ix) = rawY(Ix);
A = nan(size(rawA));
A(Ix) = rawA(Ix);
O = nan(size(rawO));
O(Ix) = rawO(Ix);

u_speed = LoadXDFDataFileByName(f, 'Speed_Command');
u_turn = LoadXDFDataFileByName(f, 'Turn_Command');
u_turn = u_turn/180*2*pi;
u_vert = LoadXDFDataFileByName(f, 'Vertical_Command');

depth = LoadXDFDataFileByName(f, 'Depth_Measurement');

heading = LoadXDFDataFileByName(f, 'Tracked_Heading');
speed = LoadXDFDataFileByName(f, 'Tracked_Speed');

% Load the camera calibration information
%
% NOTE: the blob info reported by the tracker already normalizes the
% intrinsic parameters of the camera: fc, cc, alpha_c, kc
calibration_dir = 'C:\Users\dcsl\Desktop\BelugaSupport';
CalInfo = cell(4, 1);
for q = 1 : 4,
    calibration_file = fullfile(calibration_dir, sprintf('CalibQ%d.dat', q));
    [fc, cc, alpha_c, kc, R, T, lines] = ReadCalibrationData(calibration_file);
    CalInfo{q}.f = fc;
    CalInfo{q}.c = cc;
    CalInfo{q}.a = alpha_c;
    CalInfo{q}.k = kc;
    CalInfo{q}.R = R;
    CalInfo{q}.T = T;
    
    % use normalized values
    CalInfo{q}.f = mean(fc)*[1 1];
    CalInfo{q}.c = [320 240];
    CalInfo{q}.a = 0;
    CalInfo{q}.k = [1 0 0 0 0];
    CalInfo{q}.rC = -R'*T; % camera position in world coords
end

% compute real-world coordinates from camera coordinates using camera
% calibration

X_all = nan(Nt, N*4);
Y_all = nan(Nt, N*4);
Z_all = nan(Nt, N*4);
for ix = 1 : N,
    for tx = 1 : Nt,
        for q = 1 : 4,
            cx = (ix - 1)*4 + q;
            if(~isnan(X_cam(tx, cx)))
                xc = X_cam(tx, cx);
                yc = Y_cam(tx, cx);
                
                % if the robot wasn't connected, depth will be NaN and
                % we'll assume the object was on the surface
                if isnan(depth(tx, ix)),
                    z = water_depth;
                else
                    z = water_depth - (depth(tx, ix) - depth_offset)*depth_scale;
                end
                
                h = CalInfo{q}.rC(3) - water_depth;
                rW = imageToWorld(xc, yc, CalInfo{q}.f, CalInfo{q}.c, CalInfo{q}.R, CalInfo{q}.T, water_depth-z, h);
                X_all(tx, cx) = rW(1);
                Y_all(tx, cx) = rW(2);
                Z_all(tx, cx) = rW(3);
                
            end
        end
    end
end

plot(X_all, Y_all); axis equal