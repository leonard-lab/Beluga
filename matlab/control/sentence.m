function depth = sentence(dir1, speed1, dir2, speed2, servo, s)

% Motor 1
if (speed1 > 45)
    fprintf(1, 'Value is too high, sending 45');
    speed1 = 45;
end
if (speed1 < 0)
    fprintf(1, 'Value is too low, sending 0');
    speed1 = 0;
end

% Format of string1 must be '$045!'
if (dir1 == 0) a = '$0';
elseif (dir1 == 1) a = '$1';
else
    fprintf(1, 'Dir is not valid');
    return;
end

if (speed1 < 10) b = '0';
else b = '';
end
c = num2str(speed1);
d = '!';
sendstring1 = strcat(a,b,c,d);

% Motor 2
if (speed2 > 45)
    fprintf(1, 'Value is too high, sending 45');
    speed1 = 45;
end
if (speed2 < 0)
    fprintf(1, 'Value is too low, sending 0');
    speed1 = 0;
end

% Format of string2 must be '045!'
if (dir2 == 0) a = '0';
elseif (dir2 == 1) a = '1';
else
    fprintf(1, 'Dir is not valid');
    return;
end

if (speed2 < 10) b = '0';
else b = '';
end
c = num2str(speed2);
d = '!';
sendstring2 = strcat(a,b,c,d);

% Servo
if (servo > 137)
    fprintf(1, 'Value is too high, sending 137');
    servo = 137;
end
if (servo < 0)
    fprintf(1, 'Value is too low, sending 0');
    servo = 0;
end

%Format of string must be '137!'
if (servo < 10) a = '00';
elseif (servo < 100) a = '0'; 
else a = '';
end
b = num2str(servo);
c = '!';
sendstring3 = strcat(a,b,c);

sendstring = strcat(sendstring1,sendstring2,sendstring3);
fprintf(s, sendstring);

depth = fscanf(s, '%d');