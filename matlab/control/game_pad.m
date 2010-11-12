joy = vrjoystick(1);
upcmd = zeros(1,2);
fwdcmd = zeros(1,2);
servocmd = zeros(1);

while 1
     [axes, buttons, povs] = read(joy);
     %axes = jst;
     upcmd(1) = (axes(2) > 0);
     fwdcmd(1) = (axes(4) > 0);
     upcmd(2) = floor(abs(axes(2))*40);
     fwdcmd(2) = floor(abs(axes(4))*40);
     servocmd = 68+floor(32*axes(3));
     up(upcmd(1),upcmd(2),s);
     pause(.1)
     fwd(fwdcmd(1),fwdcmd(2),s)
     pause(.1)
     servo(servocmd,s);
     pause(.1)
end