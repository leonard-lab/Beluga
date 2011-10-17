    function [y,Y,P,Y1]=ut_state(f,X,Xv,Wm,Wc,u)
    % [x1,X1,P1,X2]=ut_state(fstate,X,Xv,Wm,Wc,u);
        [n,m]=size(X);
        y=zeros(n,1);
        Y=zeros(n,m);
        for k=1:m
            Y(:,k)=f(X(:,k),u,Xv(:,k));
            y=y+Wm(k)*Y(:,k);
        end
        
        
        Y1=Y-y(:,ones(1,m));
        P=Y1*diag(Wc)*Y1';
        
    end