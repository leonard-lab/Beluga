function status = WriteCalibrationData(filename, fc, cc, alpha_c, kc, R, T)

[p, b, e] = fileparts(filename);
if ~strcmp(e, '.dat'),
    e = '.dat';
end

filename = fullfile(p,[b e]);
f = fopen(filename, 'w');

if f <= 0,
    status = -1;
    error('Could not open %s', f)
end

num_rows = 9;
row_lengths = [-1;...
    2;...
    2;...
    1;...
    5;...
    3;...
    3;...
    3;...
    3];

tag_line = sprintf('Camera calibration - %d rows w/lengths [', num_rows);
for ix = 1 : num_rows,
    tag_line = sprintf('%s %d', tag_line, row_lengths(ix));
end
tag_line = sprintf('%s] - created %s', tag_line, datestr(now));
num_format = '%f ';

fprintf(f, '%s', tag_line);
fprintf(f, '\n');
fprintf(f, num_format, fc);
fprintf(f, '\n');
fprintf(f, num_format, cc);
fprintf(f, '\n');
fprintf(f, num_format, alpha_c);
fprintf(f, '\n');
fprintf(f, num_format, kc);
fprintf(f, '\n');
fprintf(f, num_format, R(1,:));
fprintf(f, '\n');
fprintf(f, num_format, R(2,:));
fprintf(f, '\n');
fprintf(f, num_format, R(3,:));
fprintf(f, '\n');
fprintf(f, num_format, T.');
fprintf(f, '\n');

status = fclose(f);