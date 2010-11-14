% this code uses the joystick to control the beluga in the horizontal plane
% and buttons vor vertical control
joy = vrjoystick(1);
upcmd = zeros(1,2);
fwdcmd = zeros(1,2);
servocmd = zeros(1);

spd = 10; vrt = 10; rev = 10; max = 40;     % set the default speeds
dead1 = 0.00;   % set the size of the deadzone for the joystick (fwd/back)
dead2 = 0.05;   % set the size of the deadzone for the joystick (left/right)

forward = 1;

while 1
     [axes, buttons, povs] = read(joy);
     
     % the following section is for speed adjustment with button control
%     if (buttons(3)==1 && buttons(2) == 0 && spd < 40), 
%         spd = spd + 5; end
%     if (buttons(3)==0 && buttons(2) == 1 && spd > 0), 
%         spd = spd - 5; end
     
     % vertical control
     if (buttons(1)), upcmd(1) = 1; upcmd(2) = vrt; 
     else if (buttons(3)), upcmp(1) = 0; upcmd(2) = vrt;
         else upcmd(2) = 0;
         end
     end
     
     % forward/reverse control
     fwdcmd(1) = (axes(2) > 0);
     if (abs(axes(2)) < dead), fwdcmd(2) = 0;
     else if (axes(2) < 0), fwdcmd(2) = rev;
         else fwdcmd(2) = floor((abs(axes(2))-dead)*max/(1-dead));
         end
     end
     
     % servo control
     servocmd = 68-floor(32*axes(1));
     
     up(upcmd(1),upcmd(2),s);
     pause(.1)
     
     fwd(fwdcmd(1),fwdcmd(2),s);
     pause(.1)
     
     servo(servocmd,s);
     pause(.1)
end