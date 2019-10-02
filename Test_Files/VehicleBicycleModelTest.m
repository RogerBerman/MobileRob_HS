clc
clear all

addpath('../Region_Obstacle')
addpath('../Transmit')
addpath('../Motor_Funcs')
%delete any related port that is com6 from matlab workstation
delete(instrfind({'Port'},{'COM6'}))
%% Setup for Motive
dllPath = fullfile('D:','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0);
HostIP = char('127.0.0.1');
theClient.Initialize(HostIP, HostIP);

%% Initialize Arduino port
[s,flag] = initSerial_copy();     % initSerial('COM3', 6);
ppmValues = [1500,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);
pause(1)

%% Trajectory
disp('Test Start')
% Save Trajectories
X = [];
Y = [];
TIME = [];

% Sim Time
time = 0:1:100;
timeLen = length(time);
% Car length
u0 = 0.4;
u1 = 0.75;

tic

for index = 1:1:timeLen
    % Grab current Data Frame
    frameOfData = theClient.GetLastFrameOfData();
    % assign vehicle to first rigid bod
    vehicle = frameOfData.RigidBodies(1);
    
    % X & Y coordinates
    x = double(vehicle.x);
    y = double(vehicle.z);
    
    d = atan(u0*0.1/u1);
    
    ppmValues = [forwardPPM(u1),turnPPM(d + index/100),1500,1500,1500,1500];
    safeTransmit(s, ppmValues, x, y);

 

    % Collect the results:
    X = cat(1,X, x);
    Y = cat(1,Y, y);
    TIME = cat(1,TIME, toc);
    
%     pause(0.001)
end

% Stop Car
ppmValues = [1500,1500,1500,1500,1500,1500];
safeTransmit(s, ppmValues, x, y);

%% disconnect client anbd port
theClient.Uninitialize;
fclose(s);
disp('Test Done')


%% Plotting Code
state = [X Y TIME];
t = time;

figure(1)
hold on
plot(state(:,1),state(:,2))
title('Actual Position')
    xlim([-2.5 2.5])
    ylim([-1.5 1.5])
grid on
