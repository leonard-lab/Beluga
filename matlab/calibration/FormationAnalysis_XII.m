clear all;  close all;

% 2/21/11 - From scratch, using bits of BearingAnalysis_XII

data_dir = '../data/2_fish';

data_file_in = fullfile(data_dir, 'data_segments_XII.mat');


if exist(data_file_in, 'file') ~= 2,  % 2 indicates a regular file
    error(sprintf('Input file %s does not exist.\n', data_file_in));
else
    load(data_file_in, '-regexp', '[^(gen_data)|^(DO_PDFS)]')
end

if ~gen_data
    if exist(data_file_out, 'file') ~= 2,
        gen_data = 1;
    else
        load(data_file_out)
        gen_data = 0;
        DO_PDFS = 1;
    end
end

data_file_out = fullfile(data_dir, 'FormationData_XII.mat');
DO_PDFS = 0;

DILL_ANGLES = [-90 -63.4 -45 -35.3 35.3 45 63.4 90];

