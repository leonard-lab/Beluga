function success = startBelugaServer(logfile)
%
% success = startBelugaServer(logfile)
%
% Starts the beluga server, checks for a connection, and returns 1
% on success.  On error, throws an error (and returns 1).
%
% Checks to make sure the server isn't already running.

if nargin == 0,
    logfile = 'beluga_server.log';
end

pid = isBelugaServerRunning();
if pid > 0,
    error('Beluga IPC server is already running as process with PID %s', pid)
    success = 0;
    return
end

server_path = fullfile(fileparts(mfilename('fullpath')), '../beluga_server.rb');

cmd = '';
if ispc
    cmd = sprintf('start "beluag_server" ruby %s >> %s 2>&1', server_path, logfile);
else
    cmd = sprintf('ruby "%s" >> "%s" 2>&1 &', server_path, logfile);
end

fprintf('cmd = |%s|\n', cmd);
%return

[status, r] = system(cmd);
if status > 0,
    error(r)
    success = 0;
    return
end

disp('Server launched, checking connection...')
pause(1)

sock = getBelugaIPCSocket('127.0.0.1', 1234);
resp = belugaIPCMessage('ping', sock);

if ~strcmp(resp, 'PONG!')
    error('Failure to confirm server launch [got %s].', resp)
    success = 0;
    return
end

disp('Success.')
success = 1;