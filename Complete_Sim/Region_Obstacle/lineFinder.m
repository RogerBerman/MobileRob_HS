function [m, b] = lineFinder(p1, p2)
    % Find Slope
    m = (p2(2)-p1(2))/(p2(1)-p1(1));
    % Find offset
    b = p2(2) - m*p2(1);
end