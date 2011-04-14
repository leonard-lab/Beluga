function response = belugaIPCMessage(message, sock)

ServerSettings;

sock_opened_here = 0;

if(nargin == 1 || isempty(sock)),
    sock = tcpip(beluga_host, beluga_port);
end

if(strcmp(sock.Status, 'closed')),
    fopen(sock);
    fgetl(sock);  % server message
    sock_opened_here = 1;
end

fprintf(sock, sprintf('%s', strtrim(message)));
response = fgetl(sock);

if(sock_opened_here),
    fclose(sock);
end

