function [] = regionPrint(obs, region)
    % range of barrier region
    range = 3;
    % Square points around Obstacle 
    lp = [obs(1)-obs(3) obs(2)];
    rp = [obs(1)+obs(3) obs(2)];
    bp = [obs(1) obs(2)-obs(3)];
    tp = [obs(1) obs(2)+obs(3)];
    
    % Find Region
    if region == 0
        [mr ,br] = lineFinder(rp, bp);
        [ml ,bl] = lineFinder(lp, bp);
    else
        [mr ,br] = lineFinder(rp, tp);
        [ml ,bl] = lineFinder(lp, tp);
    end
    
    xr = linspace(obs(1)+range,obs(1));
    yr = mr*xr+br;
    xl = linspace(obs(1)-range,obs(1));
    yl = ml*xl+bl;
    
     % Plot Region
    plot(xr, yr)
    hold on
    plot(xl, yl)
    hold on
    
    % Plot Obstacle
    plot([lp(1) tp(1)], [lp(2) tp(2)])
    hold on
    line([rp(1) tp(1)], [rp(2) tp(2)])
    hold on
    line([lp(1) bp(1)], [lp(2) bp(2)])
    hold on
    line([rp(1) bp(1)], [rp(2) bp(2)])
end