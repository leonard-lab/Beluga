function stopBelugaServer(sock)

if nargin == 1 && isa(sock, 'tcpip'),
    belugaIPCMessage('S', sock);
    closeBelugaSocket(sock);
end