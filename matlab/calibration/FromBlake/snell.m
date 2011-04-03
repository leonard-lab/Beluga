% this function assumes that the R matrix is ~diag([-1 1 -1])

function [snell_pts] = snell(im_pts,wrld_pts,R,T,depth,opt) % for single points, pass depth rather than wrld_pts
if opt == 1, snell_pts = zeros(size(im_pts));
elseif opt == 2, snell_pts = zeros(size(wrld_pts)); end

% LOAD VARIABLES NEEDED
% load forrestal kc fc cc alpha_c
load values
if nargin < 5
    depth = input('Please input the current depth of water in the tank (m): ');
end
if isempty(depth),    depth = 2.4384;     end; % set default depth to be 8ft

H = abs(R(:,3)'*T); % world frame relative to camera frame in world units (Z-direction)
if H < 2 || H > 10
    H = 6; % guestimate of the the camera height from tank bottom
end

x_angle = 77.0; % horizontal range of camera capture (103.6 or 77.0)
y_angle = 57.7;  % vertical range of camera capture (76.6 or 57.7)

na = 1.000277; % refractive index of air (wikipedia STP)
nw = 1.333; % refractive index of water (wikipedia @20 C)

% function used to solve for theta1 in method 2
tht1 = @(h1,h2,theta1,radius) h1*tand(theta1)+h2*tand(asind(...
    sind(theta1)*na/nw))-radius;

% COMPUTATION
x_adj = fc(1); %320 / tand(x_angle/2); % compute pixel equivalent adjacent lengths or focal lengths
y_adj = fc(2); %240 / tand(y_angle/2);

for i  = 1:size(im_pts,2) % loop through image points to be corrected
    h1 = H - depth;
    if min(size(wrld_pts))==1
        h2 = wrld_pts(i);           % pass depth directly
    else
        h2 = depth - wrld_pts(3,i);     % pass world pts for transformation calibration
    end

    if h2 < 0   % point above the water level
        if opt == 1, snell_pts(:,i) = im_pts(:,i);
        elseif opt ==2, snell_pts(:,i) = wrld_pts(:,i); end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% METHOD ONE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute new pixel locations from the given feild of view (range of angles
% accross the x and y camera axes) and the given depth
    elseif opt == 1
    theta_x = atand((cc(1)-im_pts(1,i))/x_adj); % angles from optical axis
    theta_y = atand((cc(2)-im_pts(2,i))/y_adj);
    
    % create a unit vector in the C frame pointed toward the pixel
    x_temp = tand(theta_x);    y_temp = tand(theta_y);
    temp = [x_temp; y_temp; 1]/norm([x_temp; y_temp; 1]); % unit vector in C frame
    
    % recall that []c = R*[]w + T >> columns of R are w unit vectors in c
    temp_w = (R)'*temp;  % vector in the C frame dot'd with w unit vectors
    
    theta_xy = atand(temp_w(2)/(temp_w(1))); % angle in the xy plane, ccw from +x axis
    if temp_w(1) < 0
        theta_xy = theta_xy + 180;
    end
    if theta_xy < 0, theta_xy = theta_xy + 360; end % fix quadrant 4
    if isnan(theta_xy), theta_xy = 0; end           % fix center point
%     disp(['theta_x_y: ' num2str(theta_xy)]) % used in debugging
    
    theta1 = abs(atand(norm(temp_w(1:2))/temp_w(3))); % theta1 = atan(r/z) > 0
    theta2 = asind(sind(theta1)*na/nw); % by snell's law

    % calculate theta0 as the straight line to the true object position, then
    % decompose into x and y components sucht that the angle between x and y
    % remains intact from before
    theta0 = atand((h1*tand(theta1)+h2*tand(theta2))/(h1+h2)); % not used
    
    % object position relative to camera, expressed as vector in world frame
    x_cw = (h1*tand(theta1)+h2*tand(theta2))/(sqrt(1+tand(theta_xy)^2)); 
    y_cw = abs(x_cw*tand(theta_xy));
    
    if theta_xy > 180
        y_cw = -abs(y_cw);
    end
    if theta_xy > 90 && theta_xy < 270
        x_cw = -abs(x_cw);
    end
    
    new = R*[x_cw; y_cw; -(h1+h2)]; % convert new vector to camera coordinates
    new = new/norm(new);
    
    theta_x0 = atand(new(1)/abs(new(3))); % new angles from optical axis
    theta_y0 = atand(new(2)/abs(new(3)));
    
    x_temp = x_adj*tand(theta_x0); % can be combined with above step
    y_temp = y_adj*tand(theta_y0);
    
    snell_pts(1,i) = cc(1)-x_temp; % compute pixel location
    snell_pts(2,i) = cc(2)-y_temp;
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NEW METHOD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Take world position to be known absolutlely, then recompute world
% location by finding the apparent world theta1 ray (due to snell's law
% effect) and recompute pixel location from this.  This method will use H,
% h1 and h2 from previous calculations.  New variables in this section will
% be denote by a b terminal/postscript character.
    elseif opt == 2
    rel = wrld_pts(:,i) + R'*T; % Rw(p/c) [position of point from camera in world units]
%     rel/norm(rel)
%     disp(['correlation in position vectors = ' num2str(acosd(temp_w'*(rel/norm(rel))))]) % debugging

    theta_xyb = atand(rel(2)/rel(1)); % angle of rel vector in xy world plane
    if rel(1) < 0
        theta_xyb = theta_xyb + 180;
    end
    if theta_xyb < 0, theta_xyb = theta_xyb + 360; end % fix quadrant 4
    if isnan(theta_xyb), theta_xyb = 0; end            % fix center point

    theta0b = atand(norm(rel(1:2))/norm(rel(3)));
    
    theta1b = fzero(@(theta1) tht1(h1,h2,theta1,norm(rel(1:2))),theta0b);
        
    x_cwb = (h1+h2)*tand(theta1b)/(sqrt(1+tand(theta_xyb)^2)); % switch to theta0b for pixels
    y_cwb = abs(x_cwb*tand(theta_xyb));
    
    if theta_xyb > 180 % align x_cwb and y_cwb with x and y components of rel
        y_cwb = -abs(y_cwb);
    end
    if theta_xyb > 90 && theta_xyb < 270
        x_cwb = -abs(x_cwb);
    end

    % so at this point, we have a new vector [x_cwb;y_cwb;-(h1+h2)] that
    % denotes the apparent location of the world point due to distortion of
    % snell's law effects
    
    camvec = R*[x_cwb;y_cwb;-(h1+h2)]; % convert world vector to camera vector
    
    % use camvec (position of apparent point in camera units) to convert
    % world postition into pixel locations
    
    x_imb = -fc(1)*camvec(1)/abs(camvec(3))+cc(1); % compute pixel locations
    y_imb = -fc(2)*camvec(2)/abs(camvec(3))+cc(2);
    
%     snell_pts(1,i) = x_imb; % return pixel location
%     snell_pts(2,i) = y_imb;

    snell_pts(:,i) = [x_cwb;y_cwb;-(h1+h2)] - R'*T;

%     % Method for compring the two methods
%     if i == 3 || wrld_pts(3,i) > 1
%     fprintf(1,['\nSnell''s Law Applied to Point: ' num2str(i)])
%     fprintf(1,['\nCalculations with Method 1:\ttheta0 = ' num2str(theta0) '\t theta_xy = ' num2str(theta_xy) ...
%         '\t theta1 = ' num2str(theta1) ' \t x_pix = ' num2str(cc(1)-x_temp) '\t x_pix = ' num2str(cc(2)-y_temp)]);
%     fprintf(1,['\nCalculations with Method 2:\ttheta0 = ' num2str(theta0b) '\t theta_xy = ' num2str(theta_xyb) ...
%         '\t theta1 = ' num2str(theta1b) ' \t x_pix = ' num2str(x_imb) '\t x_pix = ' num2str(y_imb) '\n']);
%     end
    end
end
% clear theta* *temp* *cw* new
% clearvars -except x_angle y_angle na nw snell_pts I*
% save('values','*angle','na','nw','-append')
end

%   CURRENT STATE: run both versions of snell to compare outputs at steps
%   along the way >> create a disp prompt for each point