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

r = '';
expected_response = 'Welcome to BelugaServer, client ';

for tries = 1 : 20,
    if sock.BytesAvailable > 0
        [r, ~, msg] = fread(sock, sock.BytesAvailable);
        if isempty(msg),
            r = char(r);
            break;
        end
    end
    
    % Try getting the welcome message for the first 10 tries, then if that
    % doesn't work, we try a ping response.
    %
    % (for some reason, when I first start matlab, I don't get a response
    % from the server.  EVERY time after that it works.)
    if tries > 10,
        expected_response = 'PONG!';
        fprintf(sock, 'ping');
    end
    pause(0.1)
end


if isempty(r) || ~strncmp(r, expected_response, min(length(r), length(expected_response)))
    fclose(sock);
    error('Unable to open IPC socket |%s|.', r)
    sock = 0;
end