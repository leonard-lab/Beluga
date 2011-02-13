function IX = get_search_px(search_px, th, xc, yc, w, h, A, plus_minus)

angle_width = 30*pi/180;
pmax = 0.5*abs(exp(-i*angle_width/2) + exp(i*angle_width/2));
th = -th + pi;
if plus_minus > 0,
    f = 0;
else
    f = pi;
end

IX = [];
for ix = 1 : length(search_px),
    p = 0.5*abs(exp(i*(th + f)) + exp(i*A(ix)));
    if p < pmax,
        continue;
    end
    x = round(search_px(ix,1) + xc);
    y = round(search_px(ix,2) + yc);
    if (x <= 0) || (x > w),
        continue;
    end
    if (y <= 0) || (y > h),
        continue;
    end
    IX = [IX; x y];
end