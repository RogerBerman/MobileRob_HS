clc
clear all

addpath('../RegionObstacle')
%delete any related port that is com6 from matlab workstation
delete(instrfind({'Port'},{'COM6'}))
%% Setup for motive....no need to change this
dllPath = fullfile('D:','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0);
HostIP = char('127.0.0.1');
theClient.Initialize(HostIP, HostIP);

%% OptiTrack Test Run Hybrid Point Mass Model for trajectory
display(['OptiTrack Test Start']); 
for (i = 1:10)
    frameOfData = theClient.GetLastFrameOfData();
    rigid1 = frameOfData.RigidBodies(1);
    rigid2 = frameOfData.RigidBodies(2);
    rigid3 = frameOfData.RigidBodies(3);
end
% disconnect client
theClient.Uninitialize;
% x0 = [2; -0.5; 0];
% goal = [3; 0];
% obstacle = [1 0 1/(20*sqrt(2))];

x0 = [double(rigid1.y); double(rigid1.x); 0];
obstacle = [double(rigid2.y) double(rigid2.x) 1/(20*sqrt(2))];
goal = [double(rigid3.y); double(rigid3.x)];


findPath
postprocessing


display(['OptiTrack Test Done']); 