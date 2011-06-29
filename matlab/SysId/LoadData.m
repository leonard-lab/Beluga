clear all;  close all;

addpath('d:\dsrc\MADTraC\xdf\matlab\');

%f = ReadXDF(fullfile(getenv('USERPROFILE'), 'Desktop/test.xdf'));
f = ReadXDF('./TestData.xdf');

% look at f.DataFiles.Name for a list of files available
t = LoadXDFDataFileByName(f, 'Time');
t = t - t(1);

X = LoadXDFDataFileByName(f, 'Tracked_X');
Y = LoadXDFDataFileByName(f, 'Tracked_Y');
Z = LoadXDFDataFileByName(f, 'Tracked_Z');

u_speed = LoadXDFDataFileByName(f, 'Speed_Command');
u_turn = LoadXDFDataFileByName(f, 'Turn_Command');
u_vert = LoadXDFDataFileByName(f, 'Vertical_Command');

depth = LoadXDFDataFileByName(f, 'Depth_Measurement');