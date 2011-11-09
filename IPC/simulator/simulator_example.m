clear all;
close all;

% note, if you have trouble, you might want to do something like this:
% 1) determine if there are any existing servers running
% !ps -A | grep rhubarb
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

for t = 1 : 10
    [x, y, z] = belugaGetPositionIPC(0, sock);
    
    % execute a random walk
    go_x = x + randn(1);
    go_y = y + randn(1);
    go_z = 0;
    
    belugaSendCommandIPC(0, go_x, go_y, go_z, sock);
    
    pause(2);
end

% stop the simulator when we're done
stopBelugaSimulator