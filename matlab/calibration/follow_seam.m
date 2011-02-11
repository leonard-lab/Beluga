function [P, p_f_hist, p_b_hist] = follow_seam(r0, rlims, th, p0, I, Is)
%I = MB;
%p0 = p;
%th = SEAM_ANGLES(1);
%r0 = TANK_CENTER;
%rlims = [TANK_CENTER_RADIUS 330];

max_radius = max(rlims);
min_radius = min(rlims);

[h, w] = size(I);

line_width = 3;
se1 = strel('disk', line_width);

search_radius = 10;
search_inner_radius = 6;
se_a = strel('disk', search_radius);
se_b = strel('disk', search_inner_radius);
a = se_a.getnhood;
b = zeros(size(a));
ds = search_radius - search_inner_radius;
b([ds+1 : end-ds],[ds+1 : end-ds]) = se_b.getnhood;
b = b - a;
se_c = strel('arbitrary', b);
search_px = se_c.getneighbors;
A = zeros(length(search_px), 1);
for ix = 1 : length(A),
    A(ix) = atan2(-search_px(ix, 2), search_px(ix, 1));
end

delta = line_width/search_radius;

P = [get_neigh_px(p0, w, h, se1)];


%
for dir = [-1 1],
    
    going_forward = 1;
    p_curr = p0;
    if dir == -1,
        p_b_hist = p0;
    else        
        p_f_hist = p0;
    end
    G0 = double(I(round(p0(2)), round(p0(1))));
    
    while going_forward,
        IX = get_search_px(search_px, th, p_curr(1), p_curr(2), w, h, A, dir);
        if(isempty(IX)),
            break;
        end
        
        [nI, ~] = size(IX);
        G = zeros(nI, 1);
        Gs = zeros(nI, 1);
        for ix = 1 : nI,
            G(ix) = I(IX(ix,2), IX(ix,1));
            Gs(ix) = Is(IX(ix,2), IX(ix,1));
        end
        
        [G_best, ixb] = max(double(Gs) - double(G));
        p_best = IX(ixb(1), :);
        p_new = round(p_curr + delta*(p_best - p_curr));
        
        P = [P; get_neigh_px(p_new, w, h, se1)];
        
        r = norm(p_new - r0');
        
        if(p_new(1) <= 0 || p_new(1) > w),
            going_forward = 0;
        elseif (p_new(2) <= 0 || p_new(2) > h),
            going_forward = 0;
        elseif G_best < 2,
            going_forward = 0;
        elseif r > max_radius || r < min_radius,
            going_forward = 0;
        else
            going_forward = 1;
        end
        
        p_curr = p_new;
        if dir == -1,
            p_b_hist = [p_b_hist; p_curr];
        else
            p_f_hist = [p_f_hist; p_curr];
        end
        
    end
end
