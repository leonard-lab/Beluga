joy = vrjoystick(1);
upcmd = zeros(1,2);
fwdcmd = zeros(1,2);
servocmd = zeros(1);

spd = 10;
dead = 0.15;

forward = 1;

while 1
     [axes, buttons, povs] = read(joy);
     
    if (buttons(3)==1 && buttons(2) == 0 && spd < 40), 
        spd = spd + 5; end
    if (buttons(3)==0 && buttons(2) == 1 && spd > 0), 
        spd = spd - 5; end

     %axes = jst;
     upcmd(1) = (axes(2) < 0);
     if (abs(axes(2)) < dead), upcmd(2) = 0;
     else upcmd(2) = floor((abs(axes(2))-dead)*40/(1-dead));
     end
     
     % if (buttons(7)), forward = -1*forward; end
     % fwdcmd(1) = (forward > 0);
     if (buttons(7)), fwdcmd(2) = 10; fwdcmd(1) = 0;
     else if (buttons(1)), fwdcmd(1) = 1; fwdcmd(2) = abs(spd);
         else fwdcmd(2) = 0;
         end
     end
     
     if (~buttons(1)), fwdcmd(2) = 0; end
     
     servocmd = 68-floor(32*axes(1));
     
     up(upcmd(1),upcmd(2),s);
     pause(.1)
     
     fwd(fwdcmd(1),fwdcmd(2),s);
     pause(.1)
     
     servo(servocmd,s);
     pause(.1)
end