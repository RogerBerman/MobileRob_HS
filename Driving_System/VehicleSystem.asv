clc
clear all

addpath('../Motor_Funcs')
addpath('../Transmit_Funcs')
addpath('../Complete_Sim')
addpath('../Complete_Sim/Region_Obstacle')
addpath('../Complete_Sim/Control_Funcs')
addpath('../Complete_Sim/ToolBox_Fitting_Funcs')
addpath('../Complete_Sim/Trajectory_Funcs')
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

%% Find Steepest Descent Path
disp('Locating all elements')
for (i = 1:10)
    frameOfData = theClient.GetLastFrameOfData();
    vehicle = frameOfData.RigidBodies(1);
    obstacle = frameOfData.RigidBodies(2);
    goalio = frameOfData.RigidBodies(3);
end
disp('All elements located')

% Marker Node Layout
fNode = 2;
bNode = 1;
rNode = 4;
lNode = 3;

front_node = [vehicle.Markers(fNode).x vehicle.Markers(fNode).z];
back_node = [vehicle.Markers(bNode).x vehicle.Markers(bNode).z];
x_d = front_node(1) - back_node(1);
y_d = front_node(2) - back_node(2);
% Final term deals with atan continuity
theta_car = atan2(y_d,x_d) + 2*pi*(y_d<0);

xStart = [double(vehicle.x); double(vehicle.z); double(theta_car)];
x0 = [double(vehicle.x)+0.1; double(vehicle.z)+0.1; 1];
obstacle = [double(obstacle.x) double(obstacle.z) 1/(20*sqrt(2))];
goal = [double(goalio.x); double(goalio.z)];
goalx = goal(1);
goaly = goal(2);
simTime = 12;
testName = 'O1Jump';

disp('Generating Trajectory')
findPath
disp('Trajectory Generated')
%% Create Reference Matrix
disp('Create Reference Matrix')
% Delete q column
x(:,3) = [];

partial = [t x];
scale = 1.35;
partial_ext = extendRef(partial,1,scale);
trajTime = partial_ext(end,1);
ref = generateTrajectoryData(partial_ext,500);
ref_followed = ref;

% Plot
% GenPlotTraj

% Clear Extraneous Variables
vars = {'initial','j','J','MaxStep','obs','region','RelTol','rule','t'...
    ,'T','target','tout','x','vars'};
clear(vars{:})
disp('Finished Creating Reference Matrix')

%% Track Reference Trajectory Setup
disp('Begin Tracking Reference Trajectory')
trajLength = length(ref);

% start location
XP = [];
YP = [];
THETAP = [];
DELTA = [];
TIMEP = [];
U0 = [];
U1 = [];
eX = [];
eY = [];
eTheta =[]; 

% For checking nodes
% front_node = [vehicle.Markers(f).x vehicle.Markers(f).z]
% back_node = [vehicle.Markers(b).x vehicle.Markers(b).z]
% right_node = [vehicle.Markers(r).x vehicle.Markers(r).z]
% left_node = [vehicle.Markers(l).x vehicle.Markers(l).z]

% Find length between car axes
L = sqrt((front_node(1)-back_node(1))^2 +(front_node(1)-front_node(2))^2);
% Heading degree counter
counter = 0;
% Start System Time
tic

%% Follow Trajectory
for index = 1:1:400
    % Grab current Data Frame
    frameOfData = theClient.GetLastFrameOfData();
    % assign vehicle to first rigid bod
    vehicle = frameOfData.RigidBodies(1);
    % Finding Heading of Vehicle
    front_node = [vehicle.Markers(fNode).x vehicle.Markers(fNode).z];
    back_node = [vehicle.Markers(bNode).x vehicle.Markers(bNode).z];
    x_d = front_node(1) - back_node(1);
    y_d = front_node(2) - back_node(2);
    
    theta_car = atan2(y_d,x_d) + 2*pi*(y_d<0) + pi*2*counter;
    % IMPORTANT:Deals with looping heading angle
    % Deals with no history on first index 
    if index > 1
        % Going up condition
        if THETAP(end)-theta_car > pi
            counter = counter + 1;
            theta_car = atan2(y_d,x_d) + pi*2*(y_d<0) + pi*2*counter;
        end
        % Going Down condition
        if theta_car-THETAP(end) > pi
            counter = counter - 1;
            theta_car = atan2(y_d,x_d) + pi*2*(y_d<0) + pi*2*counter;
        end
    end
    
    % X & Y coordinates
    x = double(vehicle.x);
    y = double(vehicle.z);

%     % current time
    curTime = toc;
    
    % State Vector of Vehicle
    state = [theta_car y x];
    % Grab Current Reference matrix in time
    ref_followed = locateTraj(ref_followed,curTime-0.5);
    % Grab Current Reference Values
    XR = ref_followed(1,2);
    YR = ref_followed(1,3);
    ThetaR = ref_followed(1,4);
    wR = ref_followed(1,5);
    vR = ref_followed(1,6);
%     % Calculate Error
    [X0,X1,X2] = errorCalc(ThetaR,YR,XR,state);
%     % Input Error to Controller
    [u0,u1] = controllerPhysical(X0,X1,X2,curTime,wR,vR);
    
    % Set Car Speeds
    d = double(atan(u0*L/u1));
    forppm = forwardPPM(u1);
    turnppm = turnPPM(d);
    ppmValuesN = [forppm,turnppm,1500,1500,1500,1500];
    transmitSerial(s, ppmValuesN);

    % Stop Condition: Goal Reached Code!
    distanceToGoal = sqrt((double(front_node(1)) - goalx)^2 +(double(front_node(2)) - goaly)^2)
    if(distanceToGoal <= 0.3) 
        disp('Target Found!!!!!!!!!!!');
        ppmValues = [1500,1500,1500,1500,1500,1500];
        transmitSerial(s, ppmValues);  
        break;
    end
    
    % Collect the results for plotting:
    XP = cat(1,XP, x);
    YP = cat(1,YP, y);
    THETAP = cat(1,THETAP, theta_car);
    TIMEP = cat(1,TIMEP, curTime);
    DELTA = cat(1,DELTA, d);
    U0 = cat(1,U0,u0);
    U1 = cat(1,U1,u1);
    eX = cat(1,eX,X2);
    eY = cat(1,eY,X1);
    eTheta = cat(1,eTheta,X0); 

end

% Ensure vehicle is stopped
ppmValues = [1500,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);

disp('Finished Tracking Trajectory')
%% disconnect client and port
theClient.Uninitialize;
fclose(s);

path = 'Exp_Res/';
path = strcat(path,testName);

figure()
% Plot Marker on Inital Position
plot(xStart(1),xStart(2), '^');
hold on
% Plot Marker on Target
plot(goalx,goaly, 'x');
hold on
% Plot Obstacle
plot(obstacle(1),obstacle(2), '.', 'MarkerSize',10);
hold on
plot(XP,YP)
hold on
plot(ref(:,2),ref(:,3))
hold on 
regionPrint(obstacle, x0(3))
title('Position')
xlabel('X Coordinate (m)')
ylabel('Y Coordinate (m)')
legend('Start', 'Goal', 'Obstacle', 'Vehicle', 'Reference')
xlim([-2.5 2.5])
ylim([-2.5 2.5])
grid on
str = '_Position.png';
saveas(gcf,strcat(path,str))

figure()
plot(TIMEP,U0)
hold on
plot(ref(:,1), ref(:,6))
xlabel('Time (s)')
ylabel('Angular Velocity (r/s)')
title('Angular Velocity')
legend('Input', 'Reference')
grid on
str = '_Ang.png';
saveas(gcf,strcat(path,str))

figure()
plot(TIMEP,U1)
hold on
plot(ref(:,1), ref(:,5))
xlabel('Time (s)')
ylabel('Linear Velocity (m/s)')
title('Linear Velocity')
legend('Input', 'Reference')
grid on
str = '_Lin.png';
saveas(gcf,strcat(path,str))

figure()
plot(TIMEP, eTheta)
xlabel('Time (s)')
ylabel('\theta Error (r)')
title('Heading Error')
grid on
str = '_HeadingE.png';
saveas(gcf,strcat(path,str))

figure()
plot(TIMEP, eX)
xlabel('Time (s)')
ylabel('X Coordinate Error (m)')
title('X Position Error')
grid on
str = '_XE.png';
saveas(gcf,strcat(path,str))

figure()
plot(TIMEP, eY)
xlabel('Time (s)')
ylabel('Y Coordinate Error (m)')
title('Y Position Error')
grid on
str = '_YE.png';
saveas(gcf,strcat(path,str))
