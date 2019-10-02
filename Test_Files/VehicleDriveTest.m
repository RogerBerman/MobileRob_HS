clc
clear all

addpath('../Transmit')
%delete any related port that is com6 from matlab workstation
delete(instrfind({'Port'},{'COM6'}))

%% Car Test


display(['Car Drive Test Start']); 
% Initialize Arduino port
[s,flag] = initSerial_copy();     % initSerial('COM3', 6);
ppmValues = [1700,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);

% ppmValues = [2000,2000,1500,1500,1500,1500];
% transmitSerial(s, ppmValues);
pause(3)

ppmValues = [1500,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);

pause(1)
% close port
fclose(s);

display(['Car Drive Test Done']); 