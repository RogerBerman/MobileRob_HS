function [full] = generateTrajectoryData(partial,span)
    time = partial(:,1);
    x = partial(:,2);
    y = partial(:,3);
    % Smoooth Path
    x_smooth = smooth(x,span);
    y_smooth = smooth(y,span);
    %% Calculate Theta & Linear Velocity
    % Create Theta and Linear Arrays
    theta = [];
    linearV = [];
    counter = 0;
    for i = 2:1:length(partial)-1
        % Differences
        diff_y = y_smooth(i+1) - y_smooth(i-1);
        diff_x = x_smooth(i+1) - x_smooth(i-1);
        timeStep = time(i+1) - time(i-1);
        
        % Theta rollover
        theta_temp = atan2(diff_y,diff_x) + pi*2*(diff_y<0) + pi*2*counter;
        if i>2
            % Going up condition
            if theta(i-2)-theta_temp > pi
                counter = counter + 1;
                theta_temp = atan2(diff_y,diff_x) + pi*2*(diff_y<0) + pi*2*counter;
            end
            % Going Down condition
            if theta_temp-theta(i-2) > pi
                counter = counter - 1;
                theta_temp = atan2(diff_y,diff_x) + pi*2*(diff_y<0) + pi*2*counter;
            end
        end

        theta = cat(1,theta,theta_temp);
        % Linear Velocity
        dist = sqrt((diff_x)^2 + (diff_y)^2);
        LinVel = dist/timeStep;
        linearV = cat(1,linearV,LinVel);
        
    end
    % Trim Trajectories
    time = time(2:end-1);
    x_smooth = x_smooth(2:end-1);
    y_smooth = y_smooth(2:end-1);
    % APPLY SMOOTHING TO THETA & Lin
    theta_smooth = smooth(theta,span);
    linearV_smooth = smooth(linearV,span);
    
    %% find Angular Velocity
    % Create Angular Array
    angular = [];
    for i = 2:1:length(x_smooth)-1
        % timestep
        timeStep = time(i+1) - time(i-1);
        theta_d = theta_smooth(i+1) - theta_smooth(i-1);
        ang = theta_d/timeStep;
        angular = cat(1,angular,ang);
    end
    % smooth angular
    angular_smooth = smooth(angular,span);
    
    % Trim Trajectories
    time = time(2:end-1);
    x_smooth = x_smooth(2:end-1);
    y_smooth = y_smooth(2:end-1);
    theta_smooth = theta_smooth(2:end-1);
    linearV_smooth = linearV_smooth(2:end-1);

    
    full = [time x_smooth y_smooth theta_smooth linearV_smooth angular_smooth];
    
end