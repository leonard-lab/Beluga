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
r = fgetl(sock);

expected_response = 'Welcome to BelugaServer, client ';

if ~strcmp(r(1 : length(expected_response)), expected_response)
    error('Unable to open IPC socket.')
    fclose(sock)
    sock = 0
end