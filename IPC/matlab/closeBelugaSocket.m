function closeBelugaSocket(sock)

if nargin == 1 && isa(sock, 'tcpip'),
    fclose(sock);
end