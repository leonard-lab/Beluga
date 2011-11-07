function stopBelugaSimulator()

t = timerfind('name', 'BelugaSimTimer');
if ~isempty(t)
    stop(t);
    delete(t);
end