function p_i = find_intersection_slope_pt(m1, m2, p1, p2)

% p_i = find_intersection_slope_pt(m1, m2, p1, p2)
%
% Find the point of intersection for two lines given their slopes (m1, m2)
% and a point on each (p1, p2).  A slope of +/-inf is taken to mean a 
% vertical line.
%
% Handles the following exceptions:
%
% * Parallel lines (m1 == m2):
%    - If the points are already colinear, then the midpoint between p1 and
%        p2 is returned (even if both are vertical)
%    - If the lines are both horizontal, the intersection has x = inf and 
%        y = halfway between the two lines
%    - If the lines are both vertical, the intersection has y = inf and
%        x = halfway between the two lines
% * Correctly handles vertical lines (isinf(m1) or isinf(m2))
% * Any line component returns true to isinf: an error is raised

if sum(isinf([p1 p2])),
    error('Neither point can have an infinite component')
end

if (m1 == m2) || (isinf(m1) && isinf(m2)),
    if (p2(2)-p1(2))/(p2(1)-p1(1)) == m1,
        p_i = 0.5*(p1 + p2);
    elseif m1 == 0,
        p_i = [inf 0.5*(p1(2) + p2(2))];
    elseif isinf(m1),
        p_i = [0.5*(p1(1) + p2(1)) inf];
    else,
        p_i = [inf inf];
    end
    return
end

if isinf(m1) || isinf(m2),
    p_i = [0 0];
    if isinf(m1),
        p_i(1) = p1(1);
        p_i(2) = p2(2) - m2*(p2(1) - p1(1));
    else,  %isinf(m2)
        p_i(1) = p2(1);
        p_i(2) = p1(2) - m1*(p1(1) - p2(1));
    end
    return
end
    
A = [-1 1; -m1 m2];
b = [p1(1) - p2(1); p1(2) - p2(2)];

x = A\b;

p_i = [p1(1)+x(1) p1(2)+m1*x(1)];