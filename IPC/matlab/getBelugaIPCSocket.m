function sock = getBelugaIPCSocket(host, port)

ServerSettings;

if nargin == 1,
    port = beluga_port;
end

if nargin == 0,
    host = beluga_host;
end
    
sock = tcpip(host, port);
fopen(sock);
fgetl(sock);