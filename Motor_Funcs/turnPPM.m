%% Front Wheel Turn Function
% Set Angular Speed
function [ppm] = turnPPM(angle)
    % Car turn range -30 to 30 degrees
    % PPM values from 1000 to 2000
    % shift both so at 0 (0-60 & 0-1000)
    % convert degree to radian(0-60 = 0-1.0472)
    % scale input by 1000/1.0472 = 954.92
    % add offset of 1000 after scaling
    ppm = round((angle+0.5236)*954.92+1000);
    if ppm < 1075
%         disp('Less')
        ppm = 1075;
    end
    if ppm > 2000
%         disp('Greater')
        ppm = 2000;
    end
end