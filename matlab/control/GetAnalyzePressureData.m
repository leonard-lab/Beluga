function data = GetAnalyzePressureData(filter, filtered_points)
%Gather data - assumes the com port is not open. Opens com port, gathers a
%bunch of data points (n = filter*filter_points), and then creats subsets
%of (filter) points and does a median, mean, and mode filter over each
%subset. Plots resulting points.

%Open com port
s = serial('COM1');
s.BaudRate = 4800;
fopen(s);

data_points = filter*filtered_points;

data = zeros(data_points,1);

%Gather data points
for i = 1:data_points
    fprintf(s, 'GGGGG');
    data(i) = fscanf(s, '%d');
end

%Create new vectors with the median, mean, and mode values of every
%(filter) points:
subset = zeros(filter,1);
median_data = zeros(filtered_points,1);
mean_data = zeros(filtered_points,1);
mode_data = zeros(filtered_points,1);
for i = 0:(filtered_points - 1)
    for j = 1:filter
        subset(j) = data(i*filter + j);
    end
    median_data(i+1) = median(subset);
    mean_data(i+1) = mean(subset);
    mode_data(i+1) = mode(subset);
end

subplot(2,2,1)
plot(data)
title('Raw Data');
subplot(2,2,2)
plot(median_data);
title('Median Filtered')
subplot(2,2,3)
plot(mean_data);
title('Mean Filtered')
subplot(2,2,4)
plot(mode_data);
title('Mode Filtered')

Med = median(data)
Mean = mean(data)
Mode = mode(data)


%Close com port
fclose(s)
delete(s)
clear s
