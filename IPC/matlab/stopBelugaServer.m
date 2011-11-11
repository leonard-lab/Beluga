function stopBelugaServer(sock)

if nargin == 1 && isa(sock, 'tcpip'),
    if isBelugaServerRunning() > 0
        belugaIPCMessage('S', sock);
    end
    closeBelugaSocket(sock);
else
    if isBelugaServerRunning() > 0
        belugaIPCMessage('S');
    end
end