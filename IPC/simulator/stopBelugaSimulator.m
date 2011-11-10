function stopBelugaSimulator(sock)

t = timerfind('name', 'BelugaSimTimer');
if ~isempty(t)
    stop(t);
    delete(t);
end

if nargin > 0 && ~isempty(sock)
    fclose(sock);
end

stopBelugaServer