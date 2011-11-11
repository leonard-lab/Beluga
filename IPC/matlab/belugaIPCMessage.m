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

response = '';
for tries = 1 : 10,
    if sock.BytesAvailable > 0
        [response, ~, msg] = fread(sock, sock.BytesAvailable);
        if isempty(msg),
            response = char(response');
            break;
        end
    end
    pause(0.05)
end

if isempty(response)
    fprintf('Warning: No response from server.\n');
else
    if response(end) == 10,
        response = response([1 : end-1]);
    end
end

if(sock_opened_here),
    fclose(sock);
end

