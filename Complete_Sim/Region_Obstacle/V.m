% Lyapunov Function V
function value = V(z,goal,dist)
    x = z(1);
    y = z(2);
    xG = goal(1);
    yG = goal(2);
    % Find V value
    value = 0.5*(x-xG)^2 + 0.5*(y-yG)^2 + barrierFunc(dist);
end

% Barrier Function
function value = barrierFunc(m)
    if m > 1
        value = 0;
    else
        value = ((m-1)^2)*log(1/m);
    end
end