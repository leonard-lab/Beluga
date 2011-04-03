% fill in the correct image (Irec) by taking the weighted average of the 4
% pixels in the region of the original image, that the undistorted pixel is
% distorted to in the mapping from px >> px2
function [Irec] = Irct(I)

% SHOW THE ORIGINAL IMAGE:
quest = input('Do you want to display distorted image? (yes/no) ','s');
if strcmp(quest,'y'), quest = 'yes';    end;
if isempty(quest),    quest = 'no';     end;

if strcmp(quest,'yes'),
    figure(1);    image(I);     colormap(gray(256));
    title('Original image (with distortion) - Stored in array I');
    drawnow;
end

% UNDISTORT THE IMAGE:
fprintf(1,'Computing the undistorted image...\n')

[nr,nc] = size(I);
Irec = 255*ones(nr,nc);

load values

Irec(ind_new) = a1 .* I(ind_lu) + a2 .* I(ind_ru) +...
    a3 .* I(ind_ld) + a4 .* I(ind_rd);

fprintf(1,'done\n');

% SHOW THE UNDISTORTED IMAGE:
quest = input('Do you want to display corrected image? (yes/no) ','s');
if strcmp(quest,'y'), quest = 'yes';    end;
if isempty(quest),    quest = 'no';     end

if strcmp(quest,'yes'),
    figure(2);  image(Irec);  colormap(gray(256));
    title('Undistorted image - Stored in array I2');
    drawnow;
end


end