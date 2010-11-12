%Send Motor 2 - assumes com port is open already.

function fwd(dir, speed, s)

if (speed > 45)
    fprintf(1, 'Value is too high, sending 45');
    speed = 45;
end
if (speed < 0)
    fprintf(1, 'Value is too low, sending 0');
    speed = 0;
end

%Format of string must be '$045!'
if (dir == 0) a = '$0';
elseif (dir == 1) a = '$1';
else
    fprintf(1, 'Dir is not valid');
    return;
end

if (speed < 10) b = '0'
else b = '';
end
c = num2str(speed);
d = '!';
sendstring = strcat(a,b,c,d)
fprintf(s, sendstring);

% %Format of string must be '#045!'
% if (dir == 0) a = '%0';
% elseif (dir == 1) a = '%1';
% else
%     fprintf(1, 'Dir is not valid');
%     return;
% end
% 
% if (speed < 10) b = '0'
% else b = '';
% end
% c = num2str(speed);
% d = '!';
% sendstring = strcat(a,b,c,d)
% fprintf(s, sendstring);
