clc
clear all

addpath('../Transmit')
%delete any related port that is com6 from matlab workstation
delete(instrfind({'Port'},{'COM6'}))

%% Car Test


disp('Car Turn Function Test Start'); 
% Initialize Arduino port
[s,flag] = initSerial_copy();     % initSerial('COM3', 6);
% -30 degrees to 30 degrees
angles = -0.5236:0.1745:0.5236; 
tic
for i = 1:1:7
    angle = angles(i)
    turnVal = turnPPM(angle)
    ppmValues = [1500,turnVal,1500,1500,1500,1500];
    transmitSerial(s, ppmValues);
    disp(i-4)
    toc
    pause(0.75)
end

ppmValues = [1500,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);

pause(1)
% close port
fclose(s);

disp('Car Turn Function Test Done'); 

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