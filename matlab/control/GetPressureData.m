function data = GetPressureData(noPoints)
%Input: noPoint = an int, the number of points to gather
%Opens the com port. Sends the command 'GGGGG' to the PIC, and then gathers
%the resulting pressure data and stores it into the vector data. Plots.

%Open com port
s = serial('COM1');
s.BaudRate = 4800;
fopen(s);

data = zeros(noPoints,1);

%Gather data points
for i = 1:noPoints
    fprintf(s, 'GGGGG');
    data(i) = fscanf(s, '%d');
end

plot(data)

Med = median(data)
Mean = mean(data)
Mode = mode(data)


%Close com port
fclose(s)
delete(s)
clear s
