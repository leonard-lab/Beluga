function B = ContrastFilter(I, thresh, width)

%[w, h] = size(I);
%W = ceil((width - 1)/2);
%B = zeros(size(I));

h = fspecial('gauss', width, width/4);
h = max(max(h)) - h;
h = h/sum(sum(h));

F = imfilter(I, h);

%B = imabsdiff(I, F);
B = imsubtract(F, I);
B = (B > thresh);

% for x = 1 : w,
%     for y = 1 : h,
%         X = [max(1, x-W) : min(w, x+W)];
%         Y = [max(1, y-W) : min(h, y+W)];
%         
%         M = I(X,Y);
%         r = mean(mean(M));
%         B(x,y) = abs(I(x,y) - r)) < thresh;
%     end
% end