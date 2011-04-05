%function golqr(depth,bound,s)
opencom
pause(2)
bound = 370;
depth = 1.5;
timestep = 0.1;

result = [];

%up = [11, 12, 17, 19, 20, 23, 24, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40];
%down = [11, 11, 15, 19, 21, 24, 27, 29, 31, 34, 36, 38, 40];
up = [13, 16, 19, 20, 21, 23, 24, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40];
down = [14, 16, 18, 19, 21, 24, 27, 29, 31, 34, 36, 38, 40];


val = sentence(1,0,1,0,60,s);
curdepth = (val-bound)/234.63;
depth1 = curdepth-depth;
depth2 = curdepth-depth;
depth3 = curdepth-depth;

while (1)
   pause(timestep);
   
   z = curdepth-depth;
   %zdot = ((curdepth-depth1)/timestep+(curdepth-depth2)/(2*timestep)+(curdepth-depth3)/(3*timestep))/3;
   zdot = ((curdepth-depth1)/timestep+(curdepth-depth2)/(2*timestep))/2;
   
   x = [z; zdot];
   K = [10 4];
   u = K*x;
   
   speed = 0;
   dir = 0;
   
   % convert u to motor input values
   if u >= 0.25
       if u < 9
           speed = up(round(2*u));
       else speed = 40;
       end
       dir = 0;
   end
   
   if u < -0.25
       if u > -6
           speed = down(round(-2*u));
       else speed = 40;
       end
       dir = 1;
   end
      
   depth3 = depth2;
   depth2 = depth1;
   depth1 = curdepth;   
   val = sentence(dir,speed,1,0,60,s);
   curdepth = (val-bound)/234.63;
   
   if curdepth > 1.4
       depth = 0.5;
   end
   
   result = [result [u; speed; curdepth]];
   
   fprintf('u: %3.1f speed: %2.0f depth: %2.2f\n', u, speed, curdepth);
end