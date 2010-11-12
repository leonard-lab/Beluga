clear all;  close all;

%% MAIN CLIENT PROGRAM

MT_Path = '../../MADTraC';
addpath(fullfile(MT_Path, 'matlab/MT_MatlabClient'));

sock = MT_ServerConnect();

[s, c, d, m] = MT_GetMessageTable(sock);

msg_table = zeros(1,1);

get_blob_info = 1;

for ix = 1 : length(s),
    m{ix}
    if(strcmp(m{ix}, 'BelugaTracker')),
        msg_table(c(ix)+1) = s(ix);
    end
end

MT_InitServerExchange(sock);

w = 800;  h = 600;

figure(1)
axis([0 w 0 h])
stop = 0;
set(1, 'ButtonDownFcn', 'stop = 1;');

while ~stop

    MT_SendMappedMessage(sock, get_blob_info, msg_table);
    
    nblobs = MT_ReadInt(sock)
    if(nblobs),
        x = MT_ReadDouble(sock, nblobs);
        y = MT_ReadDouble(sock, nblobs);
        
        plot(x', y', 'x')
        axis([0 w 0 h])
        hold on
    end
    
    drawnow
    
    pause(1)

end

MT_EndServerExchange(sock);


MT_ServerDisconnect(sock);
clear sock