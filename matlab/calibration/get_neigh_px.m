function p = get_neigh_px(p0, w, h, se)

IX = se.getneighbors;
p = [];
for ix = 1 : length(IX),
    x = round(IX(ix,1) + p0(1));
    y = round(IX(ix,2) + p0(2));
    if (x <= 0) || (x > w),
        continue;
    end
    if (y <= 0) || (y > h),
        continue;
    end
    p = [p; x y];
end