function pid = isBelugaServerRunning()
%
% pid = isBelugaServerRunning()
%
% If Beluga server is running, returns its pid.  Otherwise returns 0.
%

if ispc
    [~, r] = system('netstat -anop TCP | find "1234" | find "LISTENING"');
    if isempty(r)
        pid = 0;
    else
        a = strfind(r, ' ');
        a = r([a(end)+1 : end-1]);
        pid = sscanf(a, '%d');
    end
else
    [~, r] = system('ps -A | grep beluga_server.rb | grep -v grep | cut -d'' '' -f1');
    if isempty(r)
        pid = 0;
    else
        pid = r;
    end
end