clear all;  close all;

Q = 4;

orig_file = sprintf('CalibPoints-Q%d.dat', Q);

D = load(orig_file);
[np, ~] = size(D);
K_I = D(:, [1 2]);
K_W = D(:, [3 4 5]);

% water height
h_w = 2.286;  % est 7.5'

tmp_dir = sprintf('cal_tmp_Q%d', Q);
mkdir(tmp_dir);

calib_prog = fullfile('c', 'OpenCVCalib');

first_output = fullfile(tmp_dir, 'cal00.dat');
first_pts = fullfile(tmp_dir, 'calpts00.dat');

eval(sprintf('!%s %s %s', calib_prog, orig_file, first_output));

save(first_pts, '-ascii', 'D');

A_W = zeros(size(K_W));

for ix = 1 : 20,
    
    in_cal = fullfile(tmp_dir, sprintf('cal%02d.dat', ix-1));
    in_pts = fullfile(tmp_dir, sprintf('calpts%02d.dat', ix-1));
    Di = load(in_pts);
    K_I = D(:, [1 2]);
    K_W = D(:, [3 4 5]);
    
    [~, ~, ~, ~, R, T, ~] = ReadCalibrationData(in_cal);
    
    % camera position in world frame
    C_W = -(R')*T;
    
    % height of camera above water
    h = C_W(3) - h_w;
    
    for px = 1 : np,
        % point in world coords
        P_W = K_W(px,:)';
        
        % check if point is under water
        if P_W(3) > h_w,
            A_W(px, :) = P_W';
            continue
        end
        
        % point relative to camera
        P_Cb = P_W - C_W;
        
        % depth of point
        d = h_w - P_W(3);
        
        phi = atan2(P_Cb(2), P_Cb(1));
        rP = sqrt( P_Cb(1)^2 + P_Cb(2)^2);
        rS = solveForRS(rP, d, h);
        
        S_Cb = [rS*cos(phi); rS*sin(phi); -h];
        
        S_W = S_Cb + C_W;
        
        % would be better to project to bottom of tank...
        A_W(px,:) = S_W';
        
    end
    
    DO = [K_I A_W];
    
    out_pts = fullfile(tmp_dir, sprintf('calpts%02d.dat', ix));
    out_cal = fullfile(tmp_dir, sprintf('cal%02d.dat', ix));
    save(out_pts, '-ascii', 'DO');
    
    eval(sprintf('!%s %s %s', calib_prog, out_pts, out_cal));
    
    %pause
    
end

[fc, cc, alpha_c, kc, R, T, lines] = ReadCalibrationData(out_cal);

fprintf('Extrinsic Calibration: Quadrant %d\n', Q);
disp('Final Camera Position:')
T_W = -(R')*T

disp('Final Rotation Matrix')
R

final_cal = sprintf('SnellCalib-Q%d.dat', Q);

eval(sprintf('!cp %s %s', out_cal, final_cal));
eval(sprintf('!rm -rf %s', tmp_dir));
