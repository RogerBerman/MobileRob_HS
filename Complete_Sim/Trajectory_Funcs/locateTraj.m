function [refPresent] = locateTraj(ref,sysTime)
    % Check for end of trajectory
    if length(ref) < 2
        refPresent = ref(end,:);
        return
    end
    
    % Match Time in reference path
    time = ref(:,1);
    for i = 1:1:length(time)
        if time(i) >= sysTime
            refPresent = ref(i:end,:);
            return
        end
    end
end