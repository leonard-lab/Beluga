function [fc, cc, alpha_c, kc, R, T, lines] = ReadCalibrationData(filename)

fc = [0 0];
cc = [0 0];
alpha_c = 0;
kc = [0 0 0 0 0];
R = zeros(3);
T = [0 0 0]';

[p, b, e] = fileparts(filename);
if ~strcmp(e, '.dat'),
    e = '.dat';
end

filename = fullfile(p,[b e]);
f = fopen(filename, 'r');

if f <= 0,
    status = -1;
    error('Could not open %s', f)
end

lines = cell(0);
line_no = 1;

while 1,
    line = fgetl(f);
    if ~ischar(line)
        break;
    end
    lines{line_no} = line;
    line_no = line_no + 1;
end

line1 = lines{1};

tag_line_start = 'Camera calibration - ';
tag_line_middle = ' rows w/lengths [';
num_rows_expected = 9;
row_lengths_expected = [-1 2 2 1 5 3 3 3 3]';

if ~strncmp(line1, tag_line_start, length(tag_line_start)),
    error('Unexpected beginning of file - first line is %s', line1);
end

ix = length(tag_line_start);
num_rows = sscanf(line1(ix+1 : end), '%d');
if num_rows ~= num_rows_expected || num_rows ~= length(lines),
    error('Unexpected number of rows.  Expected %d, found %d, file claims to have %d',...
        num_rows_expected, length(lines), num_rows);
end

ix = ix + length(sprintf('%d', num_rows));

if ~strncmp(line1(ix+1 : end), tag_line_middle, length(tag_line_middle)),
    error('Unexpected content in file - first line is %s', line1);
end

ix = ix + length(tag_line_middle);

[row_lengths, ~, err_msg, jx]  = sscanf(line1(ix+1:end), '%f ', num_rows);

if ~isempty(err_msg),
    error('Error reading row lengths: %s', err_msg);
end

if (numel(row_lengths) ~= numel(row_lengths_expected)) || ...
        sum(row_lengths ~= row_lengths_expected),
    error('Mismatch in the number of row elements (in header)')
end

fc =        read_parameters(2, lines, row_lengths);
cc =        read_parameters(3, lines, row_lengths);
alpha_c =   read_parameters(4, lines, row_lengths);
kc =        read_parameters(5, lines, row_lengths);
R1 =        read_parameters(6, lines, row_lengths);
R2 =        read_parameters(7, lines, row_lengths);
R3 =        read_parameters(8, lines, row_lengths);
T =         read_parameters(9, lines, row_lengths);

if numel(R1) ~= numel(R2) || numel(R2) ~= numel(R3) || numel(R3) ~= numel(T),
    error('Mismatch in size of rotation matrix or translation vector')
end

R = [R1; R2; R3];
T = T';

fclose(f);

function p = read_parameters(ix, lines, row_lengths)

[p, nr, err_msg] = sscanf(lines{ix}, '%f ');
if ~isempty(err_msg),
    error('Error reading parameters:  %s', err_msg);
end
if nr ~= row_lengths(ix),
    error('Count mismatch reading parameter %d', ix);
end
p = p.';