    function [y,P,Y1]=ut_meas(h,X,Xn,Wm,Wc,R)
        
    %[z1,P2,Z2]=ut_meas(hmeas,X1,Xn,Wm,Wc,R);
        L2=size(X,2); n=size(R,2) ;
        y=zeros(n,1);
        Y=zeros(n,L2);
        for k=1:L2
            Y(:,k)=h(X(:,k),Xn(:,k));
            y=y+Wm(k)*Y(:,k);
        end
        Y1=Y-y(:,ones(1,L2));
        P=Y1*diag(Wc)*Y1';
    end