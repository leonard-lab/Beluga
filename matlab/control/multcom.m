if ~exist('s1')      % only open the port if it's not already open

s1 = serial('COM11');
s1.BaudRate = 4800;
fopen(s1);

end

if ~exist('s2')

s2 = serial('COM12');
s2.BaudRate = 4800;
fopen(s2);

end