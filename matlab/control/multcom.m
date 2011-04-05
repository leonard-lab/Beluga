if ~exist('s1')      % only open the port if it's not already open

s1 = serial('COM1');
s1.BaudRate = 4800;
fopen(s1);

end

if ~exist('s17')

s17 = serial('COM17');
s17.BaudRate = 4800;
fopen(s17);

end