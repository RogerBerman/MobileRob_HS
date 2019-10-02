clc
clear all

addpath('../Transmit')
%delete any related port that is com6 from matlab workstation
delete(instrfind({'Port'},{'COM6'}))
%% Setup for motive....no need to change this
dllPath = fullfile('D:','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0);
HostIP = char('127.0.0.1');
theClient.Initialize(HostIP, HostIP);
%% Initialize Arduino
disp('Car Drive Test 2 Start'); 
% Initialize Arduino port
[s,flag] = initSerial_copy();     % initSerial('COM3', 6);

%% Car Drive Test Code

% Drive Forwards
ppmValues = [1600,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);

% Detect Leaving trackable area
while 1
    frameOfData = theClient.GetLastFrameOfData();
    rigid1 = frameOfData.RigidBodies(1);
    x = double(rigid1.x)
    y = double(rigid1.y)
    if x > 2 || x < -1.8
        display(['X condition']);
        break
    end
    if y > 0.2 || y < 0.14
        display(['Y condition']);
        break
    end
end
% Stop Car
ppmValues = [1500,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);


%% close port
fclose(s);
% disconnect client
theClient.Uninitialize;

display(['Car Drive Test Done']);