% this function assumes that the R matrix is ~diag([-1 1 -1])
% potential problem areas: feild of view angles, 

% For single points, pass depth rather than wrld_pts, opt1 is used to
% select the method of point undistortion and opt2 is used to select
% whether or not snell's law is considered in the transformation of image 
% points to world coordinates.  All calculations assume measurements are in
% meters.  The method for computing snell's law effects is variable method.

function [wrld_crds] = realworld(im_pts,dep,opt1,opt2,quad) 
wrld_crds = zeros(size(im_pts));
snell_pix= zeros(size(im_pts));

method = 1; % select method 1 or 2

% LOAD VARIABLES NEEDED
load values % get the four quadrants R and T matrices, +tank depth
eval(['R = R_' num2str(quad) ';']);
eval(['T = T_' num2str(quad) ';']);

% % test parameters:
% R = diag([-1 1 -1]);
% if quad==2||quad==3, T(1)=0.9398; else T(1)=-0.9398; end
% if quad==1||quad==2, T(2)=-1.6256; else T(2)=1.6256; end
% T(3) = -5.5165; T = R*T; % NOTE: T given in world vector then converted

H = abs(R(:,3)'*T); % world frame relative to camera frame in world units (Z-direction)
if H < 2 || H > 10
    H = 6; % guestimate of the the camera height from tank bottom
end

% x_angle = 77.0;  % horizontal range of camera capture (103.6 or 77.0)
% y_angle = 57.7;  % vertical range of camera capture (76.6 or 57.7)

na = 1.000277; % refractive index of air (wikipedia STP)
nw = 1.333; % refractive index of water (wikipedia @20 C)

% COMPUTATION
x_adj = fc(1); %320 / tand(x_angle/2); % compute pixel equivalent adjacent lengths or focal lengths
y_adj = fc(2); %240 / tand(y_angle/2);

for i  = 1:size(im_pts,2) % loop through image points to be corrected
    
    % method of removing distortion
    if opt1 == 1
        xu = Xrect(im_pts(2,i)+1,im_pts(1,i)+1);
        yu = Yrect(im_pts(2,i)+1,im_pts(1,i)+1);
    else
        [xu,yu] = undst_point(im_pts(1,i),im_pts(2,i),px,py,pxd,pyd,0);
    end

    h1 = H - depth;     
    h2 = dep(i);           % pass depth directly
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIRST METHOD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
if method == 1 % use the incident ray method
    theta_x = atand((cc(1)-xu)/x_adj); % angles from optical axis
    theta_y = atand((cc(2)-yu)/y_adj);
    
    % create a unit vector in the C frame pointed toward the pixel
    x_temp = tand(theta_x);    y_temp = tand(theta_y);
    temp = [x_temp; y_temp; 1]/norm([x_temp; y_temp; 1]); % unit vector in C frame
    
    % recall that []c = R*[]w + T >> columns of R are w unit vectors expressed in c
    temp_w = (R)'*temp;   % vector in the C frame dot'd with w unit vectors
    
    theta_xy = atand(temp_w(2)/(temp_w(1))); % angle in the xy plane, ccw from +x axis
    if temp_w(1) < 0
        theta_xy = theta_xy + 180;                  % fix LHP
    end
    if theta_xy < 0, theta_xy = theta_xy + 360; end % fix quadrant 4
    if isnan(theta_xy), theta_xy = 0; end           % fix center point
%     disp(['theta_x_y: ' num2str(theta_xy)]) % used in debugging
    
    theta1 = abs(atand(norm(temp_w(1:2))/temp_w(3))); % theta1 = atan(r/z) > 0
    theta2 = asind(sind(theta1)*na/nw); % by snell's law

    % calculate theta0 as the straight line to the true object position, then
    % decompose into x and y components sucht that the angle between x and y
    % remains intact from before
%     theta0 = atand((h1*tand(theta1)+h2*tand(theta2))/(h1+h2)); % not used
%     fprintf(1,['theta0 = ' num2str(theta0)]) % used for debugging
    
    % object position relative to camera, expressed as vector in world frame
    x_cw = (h1*tand(theta1)+h2*tand(theta2))/(sqrt(1+tand(theta_xy)^2)); 
    y_cw = abs(x_cw*tand(theta_xy));
    
    if theta_xy > 180
        y_cw = -abs(y_cw);
    end
    if theta_xy > 90 && theta_xy < 270
        x_cw = -abs(x_cw);
    end
    
%     fprintf(1,['\nx_cw = ' num2str(x_cw) '\ty_cw = ' num2str(y_cw)]) % debugging
   
    if opt2 == 1 || h2 < 0   % do not use snell's law
        wrld_crds(1,i) = temp_w(1)*-(h1+h2)/temp_w(3) - R(:,1)'*T;
        wrld_crds(2,i) = temp_w(2)*-(h1+h2)/temp_w(3) - R(:,2)'*T;
        wrld_crds(3,i) = -(h1+h2) - R(:,3)'*T;
    else            % use snell's law in computation
        wrld_crds(1,i) = x_cw - R(:,1)'*T;      Xwn1 = x_cw - R(:,1)'*T;
        wrld_crds(2,i) = y_cw - R(:,2)'*T;      Ywn1 = y_cw - R(:,2)'*T;
        wrld_crds(3,i) = -(h1+h2) - R(:,3)'*T;  Zw1 = -(h1+h2) - R(:,3)'*T;
    end
    
    % compute pixel resultant pixel locations
    cam = R*wrld_crds(:,i)+T;
    snell_pix(1,i) = cc(1) - x_adj*cam(1)/cam(3);
    snell_pix(2,i) = cc(2) - y_adj*cam(2)/cam(3);
   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SECOND METHOD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif method == 2 % use algebraic solve method
    Zw = -(h1+h2) - R(:,3)'*T; % world coordinate Z
    Tw = R'*T;   % position vector of I relative to C in world units

    d1 = (xu - cc(1))*R(3,2)+fc(1)*R(1,2); % denominators used in calcualtions
    d2 = (yu - cc(2))*R(3,2)+fc(2)*R(2,2);

    if R(3,2)==0 && R(1,2)==0 % simply solve for Xw then Yw
        Xw = (-fc(1)*(R(1,3)*Zw+T(1))-(xu-cc(1))*(R(3,3)*Zw+T(3)))/(fc(1)*R(1,1)+(xu-cc(1))*R(3,1));
        Yw = (-fc(2)*(R(2,1)*Xw+R(2,3)*Zw+T(2))-(yu-cc(2))*(R(3,1)*Xw+R(3,3)*Zw+T(3)))/(fc(2)*R(2,2)+(yu-cc(2))*R(3,2));
    elseif R(2,1)==0 && R(3,1)==0 % ismply solve for Yw then Xw
        Yw = (-fc(2)*(R(2,3)*Zw+T(2))-(yu-cc(2))*(R(3,3)*Zw+T(3)))/(fc(2)*R(2,2)+(yu-cc(2))*R(3,2));
        Xw = (-fc(1)*(R(1,2)*Yw+R(1,3)*Zw+T(1))-(xu-cc(1))*(R(3,2)*Yw+R(3,3)*Zw+T(3)))/(fc(1)*R(1,1)+(xu-cc(1))*R(3,1));

    else    % full solve for Xw and then Yw
        Xw = ((-fc(1)*(R(1,3)*Zw+T(1)) - (xu-cc(1))*(R(3,3)*Zw+T(3)))*d2 +...
            (fc(2)*(R(2,3)*Zw+T(2)) + (yu-cc(2))*(R(3,3)*Zw+T(3)))*d1)/...
            (d2*(fc(1)*R(1,1)+(xu-cc(1))*R(3,1))-d1*(fc(2)*R(2,1)+(yu-cc(2))*R(3,2)));
        Yw = (-fc(1)*(R(1,1)*Xw+R(1,3)*Zw+T(1))-(xu-cc(1))*(R(3,1)*Xw+R(3,3)*Zw...
            +T(3)))/d1;
    end

    theta1b = atand(norm([(Xw+Tw(1));(Yw+Tw(2))])/abs(Zw+Tw(3)));
    theta2b = asind(sind(theta1b)*na/nw);

    r1 = sqrt((Xw+Tw(1))^2+(Yw+Tw(2))^2);
    del_r = dep(i) * (tand(theta2b) - tand(theta1b));
    r2 = r1 + del_r;

    a = (Yw+Tw(2))/(Xw+Tw(1)); % tan(theta_xy)

    Xwn = sign(Xw+Tw(1))*r2/sqrt(1+a^2) - Tw(1);
    Ywn = sign(Yw+Tw(2))*r2/sqrt(1+a^-2)- Tw(2);

    if opt2 == 1 || dep(i) < 0   % do not use snell's law
        wrld_crds(1,i) = Xw;
        wrld_crds(2,i) = Yw;
        wrld_crds(3,i) = Zw;
    else            % use snell's law in computation
        wrld_crds(1,i) = Xwn;
        wrld_crds(2,i) = Ywn;
        wrld_crds(3,i) = Zw;
    end

    %compute pixel locations
    xp = -fc(1)*(R(1,1)*Xwn+R(1,2)*Ywn+R(1,3)*Zw+T(1))/(R(3,1)*Xwn+R(3,2)...
        *Ywn+R(3,3)*Zw+T(3)) + cc(1);
    yp = -fc(2)*(R(2,1)*Xwn+R(2,2)*Ywn+R(2,3)*Zw+T(2))/(R(3,1)*Xwn+R(3,2)...
        *Ywn+R(3,3)*Zw+T(3)) + cc(2);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Compare Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This section is used to print out the results (with snell's law taken
% into account) for the two possible methods of computation.  To compute
% both methods supress the elseif statement in line 114 (method 2 results
% will be returned in this case)
%     disp('The Results from the method 1 are: ') % used in debugging
%     fprintf(1,['Xwn = ' num2str(Xwn1) '\tYwn = ' num2str(Ywn1) '\tZwn = ' ...
%         num2str(Zw1) '\t distance from center = ' num2str(norm([Xwn1;Ywn1;Zw1])) ]) % debugging
%     fprintf(1,['\nxp = ' num2str(snell_pix(1,i)) '\typ = ' num2str(snell_pix(2,i)) '\n\n'])
%     
%     disp('The Results from the new method are: ') % used in debugging
%     fprintf(1,['Xwn = ' num2str(Xwn) '\tYwn = ' num2str(Ywn) '\tZwn = ' num2str(Zw)...
%                 '\t distance from center = ' num2str(norm([Xwn;Ywn;Zw]))]) % debugging
%     fprintf(1,['\nxp = ' num2str(xp) '\typ = ' num2str(yp)])

end

end