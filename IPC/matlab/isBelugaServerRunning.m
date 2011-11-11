function pid = isBelugaServerRunning()
%
% pid = isBelugaServerRunning()
%
% If Beluga server is running, returns its pid.  Otherwise returns 0.
% Not yet implemented in Windows - always returns 0.
%

if ispc
    disp('Warning.  isBelugaServerRunning is not yet implemented for Windows.  Will return false.')
    pid = 0;
else
    [~, r] = system('ps -A | grep beluga_server.rb | grep -v grep | cut -d'' '' -f1');
    if isempty(r)
        pid = 0;
    else
        pid = r;
    end
end