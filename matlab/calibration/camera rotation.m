% camera relocation
clc;
offset = 180/14; %degrees off from parallel to walls

radius = (37^2 + 64^2)^0.5;

alpha = atand(37/64);

x1_0 = radius*cosd(offset+alpha);
y1_0 = radius*sind(offset+alpha);

delta = 360/14;

x1 = radius*cosd(offset+alpha+delta);
y1 = radius*sind(offset+alpha+delta);

x_move = x1-x1_0;
y_move = y1-y1_0;

x4 = radius*cosd(offset-alpha+delta);
y4 = radius*sind(offset-alpha+delta);

disp(sprintf('%s %2d', 'y1 = :',  y1 - 67.5, ' x4 = ', 85-x4))


% recorded measurements:
% Large Beam  - 21"*8" @ y = 52.5"  >>  y1 >= 67.5
% Small Beams - 12"x4" @ x = -89.0", x = 1.25", x = 91" >>  x4 <= 85"

%%
clc
x_center = 188.688;
y_center = 188.938;

top_beam = 227.25 - y_center - (48.5 + 8.5/2);

% first number is the distance from left wall to beam center
beam1 =  95.5 - x_center - (-91);
beam2 = 193.0 - x_center - (-1.25);
beam3 = 290.5 - x_center - (89);

x1 = x_center - 91  -2
x2 = x_center - 1.25-2
x3 = x_center + 89  -2