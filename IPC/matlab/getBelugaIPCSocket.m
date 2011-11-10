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

if ~strcmp(sock.Status, 'open')
    error('Unable to open IPC socket.')
    sock = 0;
    return
end

r = fgetl(sock);

expected_response = 'Welcome to BelugaServer, client ';

if isempty(r) || ~strncmp(r, expected_response, min(length(r), length(expected_response)))
    error('Unable to open IPC socket.')
    fclose(sock);
    sock = 0;
end