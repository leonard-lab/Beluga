% Send Servo Value - assumes com port is already open. Sends value to servo
% that is between 0 and 137.
function servo(x, s)

if (x > 137)
    fprintf(1, 'Value is too high, sending 137');
    x = 137;
end
if (x < 0)
    fprintf(1, 'Value is too low, sending 0');
    x = 0;
end

%Format of string must be '#137!'
if (x < 10) a = '#00';
elseif (x < 100) a = '#0'; 
else a = '#';
end
b = num2str(x);
c = '!';
sendstring = strcat(a,b,c);
fprintf(s, sendstring);