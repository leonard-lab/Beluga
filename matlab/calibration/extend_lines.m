function M = extend_lines(M, w, h)

[nlines, n_e] = size(M);

if n_e < 6,
    error('extend_lines expects data with at least 6 cols');
end

if nlines > 2,
    error('extend_lines works with no more than 2 lines at a time');
end

% nlines = 1 -> extend line to the closest edge of the image
if nlines == 1,
    p1 = M([2 3]);
    p2 = M([4 5]);
    
    % slope of the line
    m = line_slope(p1, p2);
    % midpoint
    p0 = 0.5*(p1 + p2);
    
    P = [];
    p_top = find_intersection_slope_pt(m, 0, p0, [0 0]);
    if in_frame(p_top, w, h),
        P = p_top;
    end
    p_bot = find_intersection_slope_pt(m, 0, p0, [0 h]);
    if in_frame(p_bot, w, h),
        P = [P; p_bot];
    end
    p_left = find_intersection_slope_pt(m, inf, p0, [0 0]);
    if in_frame(p_left, w, h),
        P = [P; p_left];
    end
    p_right = find_intersection_slope_pt(m, inf, p0, [w 0]);
    if in_frame(p_right, w, h),
        P = [P; p_right];
    end
    
    [np, ~] = size(P);
    d = sum( (P - repmat(p0, [np 1])).^2, 2);
    
    [dm, ix] = min(d);
    p_new = P(ix, :);
    d1 = sum((p1 - p_new).^2);
    d2 = sum((p2 - p_new).^2);
    
    if d1 < d2,
        M([2 3]) = p_new;
    else
        M([4 5]) = p_new;
    end
    
else, % n_lines == 2
    p11 = M(1, [2 3]);
    p12 = M(1, [4 5]);
    p21 = M(2, [2 3]);
    p22 = M(2, [4 5]);
    
    m1 = line_slope(p11, p12);
    m2 = line_slope(p21, p22);
    
    p_int = find_intersection_slope_pt(m1, m2, p12, p22);
    
    d1 = sum((p_int - p11).^2);
    d2 = sum((p_int - p12).^2);
    if d1 < d2,
        M(1, [2 3]) = p_int;
    else
        M(1, [4 5]) = p_int;
    end
    d1 = sum((p_int - p21).^2);
    d2 = sum((p_int - p22).^2);
    if d1 < d2,
        M(2, [2 3]) = p_int;
    else
        M(2, [4 5]) = p_int;
    end
end

function m = line_slope(p1, p2)

% handles vertical lines by returning infinite slope
m = (p2(2) - p1(2))/(p2(1) - p1(1));

function b = in_frame(p, w, h)

b = (p(1) >= 0) && (p(1) <= w) && (p(2) >= 0) && (p(2) <= h);
