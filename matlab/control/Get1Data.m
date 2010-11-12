%Get 1 data point - assume com port is open.
function data = Get1Data(s)
fprintf(s, 'GGGGG');
data = fscanf(s, '%d');
