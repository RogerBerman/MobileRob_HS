%% Set Linear Speed
function [ppm] = forwardPPM(speed)
    m = 97.6893;
    b = 1577.3;
    ppm  = round(m*speed + b);
    if ppm < 1600
%         disp('Less')
        ppm = 0;
    end
    if ppm > 1700
%         disp('Greater')
        ppm = 1700;
    end
end