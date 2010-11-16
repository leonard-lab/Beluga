% this code uses the joystick to control the beluga in the horizontal plane
% and buttons vor vertical control
joy = vrjoystick(1);
upcmd = zeros(1,2);
fwdcmd = zeros(1,2);
servocmd = zeros(1);

spd = 10; vrt = 15; rev = 20; max = 40;     % set the default speeds
dead1 = 0.00;   % set the size of the deadzone for the joystick (fwd/back)
dead2 = 0.05;   % set the size of the deadzone for the joystick (left/right)

forward = 1;

while 1
     [axes, buttons, povs] = read(joy);
     
     % throttle max horizontal velocity with dial
     % max = 20*abs(axes(3)-1);
     
     % the following section is for speed adjustment with button control
    if (buttons(6)== 1 && buttons(7) == 0 && vrt < 40), 
        vrt = vrt + 5; end
    if (buttons(6)== 0 && buttons(7) == 1 && vrt > 0), 
        vrt = vrt - 5; end
     
     % vertical control
     if (buttons(1)), upcmd(1) = 0; upcmd(2) = vrt; 
     else if (buttons(3)), upcmd(1) = 1; upcmd(2) = vrt;
         else upcmd(2) = 0;
         end
     end
     
     % forward/reverse control
     fwdcmd(1) = (axes(2) < 0);
     if (abs(axes(2)) < dead1), fwdcmd(2) = 0;
     else if (axes(2) > 0 && abs(axes(2)-dead1)/(1-dead1) > rev /max), 
             fwdcmd(2) = rev;
         else fwdcmd(2) = floor((abs(axes(2))-dead1)*max/(1-dead1));
         end
     end
     
     % servo control
     servocmd = 68-floor(40*axes(1));
     
     up(upcmd(1),upcmd(2),s);
     pause(.1)
     
     fwd(fwdcmd(1),fwdcmd(2),s);
     pause(.1)
     
     servo(servocmd,s);
     pause(.1)
end