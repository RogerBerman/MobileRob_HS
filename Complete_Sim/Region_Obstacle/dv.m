function result = dv(x, obs, target)
    % check distance
    curDis = sqrt((x(1)-obs(1))^2 + (x(2)-obs(2))^2);
    % if distance larger then 0.5 no barrier
    if curDis > 1
        bx1 = 0;
        bx2 = 0;
    else
        % Calculation of Barrier Derivative
        dbx1 = @(a,c) ((((a - x(1))^2 + (c - x(2))^2)^(1/2) - 1)^2* ...
            (2*a - 2*x(1)))/(2*((a - x(1))^2 + (c - x(2))^2)) - ...
            (log(1/((a - x(1))^2 + (c - x(2))^2)^(1/2))*(((a - x(1))^2 + ...
            (c - x(2))^2)^(1/2) - 1)*(2*a - 2*x(1)))/((a - x(1))^2 + ...
            (c - x(2))^2)^(1/2);
        
        dbx2 = @(a,c) ((((a - x(1))^2 + (c - x(2))^2)^(1/2) - 1)^2* ...
            (2*c - 2*x(2)))/(2*((a - x(1))^2 + (c - x(2))^2)) - ...
            (log(1/((a - x(1))^2 + (c - x(2))^2)^(1/2))*(((a - x(1))^2 + ...
            (c - x(2))^2)^(1/2) - 1)*(2*c - 2*x(2)))/((a - x(1))^2 + ...
            (c - x(2))^2)^(1/2);
        

        bx1 = dbx1(obs(1), obs(2));

        bx2 = dbx2(obs(1), obs(2));
    end

    % Gradient Descent
    dvx1 = x(1) - target(1) + bx1;
    dvx2 = x(2) - target(2) + bx2;
    result = -1*[dvx1; dvx2];

end