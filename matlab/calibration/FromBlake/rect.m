function [Xrect,Yrect] = rect()

load values

% BUILD XRECT AND YRECT
Xrect = zeros(nr,nc);
Yrect = zeros(nr,nc);

bad_pix = 0;

for i = 0:nc-1
    for j = 0:nr-1
        
        xsearch = i+buf;
        ysearch = j+buf;
        
        [x_temp,y_temp] = undst_point(xsearch,ysearch,px,py,pxd,pyd,buf);
        
        
        if ((x_temp == 0) &&(y_temp ==0))
%             fprintf('This point could not be computed\n')
            bad_pix = bad_pix + 1;
%             disp(['the number of bad pixels is: ' num2str(bad_pix)])
        else
%             fprintf(1,['x-distorted = ' num2str(i) ' y-distorted = ' ...
%                 num2str(j) ' corresponds to, x-true = ' num2str(x_temp)...
%                 ' y-true = ' num2str(y_temp) ' \n']);
            Xrect(j+1,i+1) = x_temp;
            Yrect(j+1,i+1) = y_temp;
            
        end
        
        if (i+1)/5 == round((i+1)/5) && j == (nr-1)
            disp(['Finished number of collumns: ' num2str(i)])
            if bad_pix > 0
                disp(['the number of bad pixels is: ' num2str(bad_pix)])
            end
        end
        
    end
end
end