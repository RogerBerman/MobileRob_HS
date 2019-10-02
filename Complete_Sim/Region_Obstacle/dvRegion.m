function result = dvRegion(z, square, goal, region)
    curDis = distFind(z, square, region);
    % if distance larger then no barrier
    if curDis(1) > 1
        bx1 = 0;
        bx2 = 0;
    else
        % Distance & Barrier derivatives for coordinates
        derX1 = @(a,b,c) (2*a*log((a^2 + b^2)^(1/2)/abs(c + a*z(1) + b*z(2)))* ...
            sign(c + a*z(1) + b*z(2))*(abs(c + a*z(1) + b*z(2))/...
            (a^2 + b^2)^(1/2) - 1))/(a^2 + b^2)^(1/2) - ...
            (a*sign(c + a*z(1) + b*z(2))*(abs(c + a*z(1) + b*z(2))/...
            (a^2 + b^2)^(1/2) - 1)^2)/abs(c + a*z(1) + b*z(2));
        
        derX2 = @(a,b,c) (2*b*log((a^2 + b^2)^(1/2)/abs(c + a*z(1) + b*z(2)))* ...
            sign(c + a*z(1) + b*z(2))*(abs(c + a*z(1) + b*z(2))/...
            (a^2 + b^2)^(1/2) - 1))/(a^2 + b^2)^(1/2) - ...
            (b*sign(c + a*z(1) + b*z(2))*(abs(c + a*z(1) + b*z(2))/...
            (a^2 + b^2)^(1/2) - 1)^2)/abs(c + a*z(1) + b*z(2));

        % Barrier derivative
        bx1 = derX1(curDis(2),curDis(3),curDis(4));
        bx2 = derX2(curDis(2),curDis(3),curDis(4));
    end
    % Gradient Descent
    dvx1 = z(1) - goal(1) + bx1;
    dvx2 = z(2) - goal(2) + bx2;
    result = -1*[dvx1; dvx2];
end