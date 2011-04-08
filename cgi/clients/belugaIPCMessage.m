function response = belugaIPCMessage(message)

host = '127.0.0.1';
port = 1234;



sock = tcpip(host, port);
fopen(sock);
fgetl(sock);
fprintf(sock, sprintf('%s\n', strtrim(message)));
response = fgetl(sock);
fclose(sock);

