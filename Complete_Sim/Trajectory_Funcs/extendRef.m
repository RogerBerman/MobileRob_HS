function [extended_ref] = Extend_Ref(ref, mag, scale)
    time = ref(:,1);
    x = ref(:,2);
    y = ref(:,3);
    traj_length = length(ref);
    
    % New Trajectories
    new_time = [];
    new_x = [];
    new_y = [];
    time_tracker = 0;
    for i = 1:1:traj_length-1
        d_time = time(i+1) - time(i);
        d_x = x(i+1) - x(i);
        d_y = y(i+1) - y(i);
        
        % Add Current element to new trajectory
        new_time = cat(1,new_time,time_tracker);
        new_x = cat(1,new_x,x(i));
        new_y = cat(1,new_y,y(i));
        dist = sqrt((d_x^2) + (d_y^2));
        
        % if enough to add data points
        if (dist > 5e-04) && (d_time >=0.0005)
            % Calculate Steps
            x_step = d_x/mag; 
            y_step = d_y/mag;
            
            % create new data points
            for j = 1:1:mag
                time_tracker = time_tracker + 0.001*scale;
                new_time = cat(1,new_time,time_tracker);
                new_x = cat(1,new_x,x(i) + x_step*j);
                new_y = cat(1,new_y,y(i) + y_step*j);
                
            end  
        end 
        time_tracker = time_tracker + 0.001*scale;
    end
    
    extended_ref = [new_time new_x new_y];


end