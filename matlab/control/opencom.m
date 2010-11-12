if ~exist('s')      % only open the port if it's not already open

s = serial('COM1');
s.BaudRate = 4800;
fopen(s);

end