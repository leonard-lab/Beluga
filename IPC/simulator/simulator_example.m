clear all;
close all;

% note, if you have trouble, you might want to do something like this:
% 1) determine if there are any existing servers running
% !ps -A | grep beluga
% 2) if the results include something like 'ruby /the/path/to/rhubarb.rb..'
% then do
% !kill pid
% where 'pid' is the process id of the running rhubarb process.

% make sure that there are no existing simulator timers running
delete(timerfind)

% launch the simulator
disp('Launching Beluga Simulator')
belugaSimulator
% wait one second to make sure the simulator launched (may not be
% necessary)
pause(1)

disp('Connecting to IPC server')
% make sure we've killed any existing socket connection
if(exist('sock'))
   clear 'sock' 
end
sock = getBelugaIPCSocket('127.0.0.1',1234);
sock.Timeout = 0.5;
sock.Name = 'ClientSocket';

% uncomment if you want verbose output in the simulator log file
%belugaIPCMessage('set verbose on');

TankRadius = 3.0; %roughly
WaterDepth = 2.286; %pretty close

for t = 1 : 10
    [X, Y, Z] = belugaGetPositionIPC([0 : 3], sock);
    
    % go to a random location in the tank
    TH = 2*pi*rand(4,1);
    R = TankRadius*rand(4,1);
    GO_X = R.*cos(TH);
    GO_Y = R.*sin(TH);
    GO_Z = WaterDepth*rand(4,1);

    belugaSetWaypointIPC([0 : 3], GO_X, GO_Y, GO_Z, sock);
    
    pause(5);
end

% stop the simulator when we're done
stopBelugaSimulator