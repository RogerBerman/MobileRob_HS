function [distance] = distFind(z, square, wedge) 
    lp = square(1,:);
    rp = square(2,:);
    tp = square(3,:);
    bp = square(4,:);
    
    if wedge == 0
        % Find Region right line
        [ar ,cr] = lineFinder(rp, bp);
        % Find Region left line
        [al ,cl] = lineFinder(lp, bp);
    elseif wedge == 1
        % Find Region right line
        [ar ,cr] = lineFinder(rp, tp);
        % Find Region left line
        [al ,cl] = lineFinder(lp, tp);
    end
    
    right = [ar cr];
    left = [al cl];
    distance = distanceToRegion(z, wedge, square, left, right);
    
end

function [dist] = distanceToRegion(z, wedge, square, left, right)
    yr = right(1)*z(1)+right(2);
    yl = left(1)*z(1)+left(2);
    if wedge == 1
        if (yr >= z(2)) && (yl < z(2))
            % find distance between the line and point(wiki function)
            top = abs(left(1)*z(1)-z(2)+left(2));
            bot = sqrt(left(1)^2 + 1);
            dist = [top/bot, left(1), -1, left(2)];
        elseif (yr < z(2)) && (yl < z(2))
            % Distance from top point
            d = sqrt((z(1)-square(3,1))^2+(z(2)-square(3,2))^2);
            dist = [d, 0, -1, square(3,2)];
%         elseif (yr < z(2)) && (yl >= z(2))
        else
            top = abs(right(1)*z(1)-z(2)+right(2));
            bot = sqrt(right(1)^2 + 1);
            dist = [top/bot, right(1), -1, right(2)];    
        end
    else
        if (yr <= z(2)) && (yl > z(2))
            % find distance between the line and point(wiki function)
            top = abs(left(1)*z(1)-z(2)+left(2));
            bot = sqrt(left(1)^2 + 1);
            dist = [top/bot, left(1), -1, left(2)];
        elseif (yr > z(2)) && (yl > z(2))
            % Distance from top point
            d = sqrt((z(1)-square(3,1))^2+(z(2)-square(3,2))^2);
            dist = [d, 0, -1, square(3,2)];
%         elseif (yr > z(2)) && (yl <= z(2))
        else
            top = abs(right(1)*z(1)-z(2)+right(2));
            bot = sqrt(right(1)^2 + 1);
            dist = [top/bot, right(1), -1, right(2)];    
        end
    end
end

% function [dist] = distanceToRegion(z, square, left, right)
%     % Distance from left region
%     if z(1)<square(3,1)
%         % find distance between the line and point(wiki function)
%         top = abs(left(1)*z(1)-z(2)+left(2));
%         bot = sqrt(left(1)^2 + 1);
%         dist = [top/bot, left(1), -1, left(2)];
%     % Distance from bottom point
%     elseif z(1) == square(3,1)
%         d = sqrt((z(2)-square(3,2))^2);
%         dist = [d, 0, -1, square(3,2)];
%     % Distance from right region
%     else
%         % find distance between the line and point(wiki function)
%         top = abs(right(1)*z(1)-z(2)+right(2));
%         bot = sqrt(right(1)^2 + 1);
%         dist = [top/bot, right(1), -1, right(2)];
%     end
% 
% end