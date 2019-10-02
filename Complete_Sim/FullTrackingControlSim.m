clc
clear all

addpath('ODE_Solvers')
addpath('Region_Obstacle')
addpath('Control_Funcs')
addpath('Trajectory_Funcs')
addpath('ToolBox_Fitting_Funcs')
%% Run Hybrid Point Mass Model for trajectory
% [x; y; wedge]
testName = 'O0Jump';
x0 = [1.9; 1.25; 0];
goal = [-1; 1];
obstacle = [1; 0.25; 1/(20*sqrt(2))];
simTime = 12;
findPath

% Delete hybrid q column
x(:,3) = [];

partial = [t x];
scale = 1.2;
partial_ext = extendRef(partial,2,scale);
trajTime = partial_ext(end,1);
ref = generateTrajectoryData(partial_ext,500);
% refU = generateTrajectoryData(partial,1);
% Plot generatedTrajectory
% GenPlotTraj
simRef = ref;

% Remove unnecessary simulation variables 
vars = {'initial','j','J','MaxStep','obs','region','RelTol','rule','t'...
    ,'T','target','tout','x','vars'};
clear(vars{:})

%% Solve ODE at fixed timesteps based on Reference Trajectory

% Grab u0,u1 values
global input0 input1 dVal L eX eY eTheta
input0 = [];
input1 = [];
L = 0.35;

dVal = [];

% start location
% [heading; y; x]
x = [pi 1.15 2.15];
THETA = [];
Y = [];
X = [];
TIME = [];
eX = [];
eY = [];
eTheta =[]; 

% Init time and control loop speed
sysTime = 0;
control_time = 0.05;
% Solve ODE at each timestep
for index = 1:control_time:trajTime
    start = [x(end,1) x(end,2) x(end,3)];
    t = [sysTime sysTime+control_time];
    simRef = locateTraj(simRef,sysTime);
    x = ode2(@(t,x) sys(t,x,simRef(1,2:6)), t , start);
    
    % Collect the results:
    THETA = cat(1,THETA, x(end,1));
    Y = cat(1,Y, x(end,2));
    X = cat(1,X, x(end,3));
    TIME = cat(1,TIME, sysTime);
    
    % Increment Time
    sysTime = sysTime+control_time;

end

% Plotting Code
t = sysTime;

% Plot Locations
path = 'Sim_Res/';
path = strcat(path,testName);

figure()
% Plot Marker on Inital Position
plot(x0(1),x0(2), '^');
hold on
% Plot Marker on Target
plot(goal(1),goal(2), 'x');
hold on
% Plot Obstacle
plot(obstacle(1),obstacle(2), '.', 'MarkerSize',10);
hold on
% Plot Reference Path
plot(ref(:,2), ref(:,3))
hold on
% Plot Vehicle Path
plot(X,Y)
hold on 
regionPrint(obstacle, x0(3))
title('Position')
xlabel('X Coordinate (m)')
ylabel('Y Coordinate (m)')
legend('Start', 'Goal', 'Obstacle', 'Reference', 'Vehicle')
xlim([-2 4])
ylim([-2 2])
grid on
str = '_Position.png';
saveas(gcf,strcat(path,str))

figure()
plot(0:TIME(end-1)/(length(input0)-1):TIME(end-1),input0)
hold on
plot(ref(:,1), ref(:,6))
xlabel('Time (s)')
ylabel('Angular Velocity (r/s)')
title('Angular Velocity')
legend('Input', 'Reference')
grid on
str = '_Ang.png';
saveas(gcf,strcat(path,str))
% 
figure()
plot(0:TIME(end-1)/(length(input1)-1):TIME(end-1),input1)
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
plot(0:TIME(end-1)/(length(eTheta)-1):TIME(end-1),eTheta)
xlabel('Time (s)')
ylabel('\theta Error (r)')
title('Heading Error')
grid on
str = '_HeadingE.png';
saveas(gcf,strcat(path,str))

figure()
plot(0:TIME(end-1)/(length(eX)-1):TIME(end-1),eX)
xlabel('Time (s)')
ylabel('X Coordinate Error (m)')
title('X Position Error')
grid on
str = '_XE.png';
saveas(gcf,strcat(path,str))

figure()
plot(0:TIME(end-1)/(length(eY)-1):TIME(end-1),eY)
xlabel('Time (s)')
ylabel('Y Coordinate Error (m)')
title('Y Position Error')
grid on
str = '_YE.png';
saveas(gcf,strcat(path,str))

%% Functions
function  xdot = sys(t, x, ref)
    global input0 input1 dVal L eX eY eTheta
    %set Values
    
    XR = ref(1);
    YR = ref(2);
    ThetaR = ref(3);
    vR = ref(4);
    wR = ref(5);
    % Calculate Error
    [X0,X1,X2] = errorCalc(ThetaR,YR,XR,x);
    % Input Error to Controller
    [u0,u1] = controller(X0,X1,X2,t,wR,vR);
    % Collect the results:
    eX = cat(1,eX,X2);
    eY = cat(1,eY,X1);
    eTheta = cat(1,eTheta,X0); 
    input0 = cat(1,input0, u0);
    input1 = cat(1,input1, u1);
    % Stabilization Guards
    if abs(u0) < 0.005
        u0 = 0;
    end
    if abs(u1) < 0.001
        u1 = 0.001;
    end
    % Dubins Model Differential Equations
    theta = x(1);
    %     Bicycle Model
    d = atan(u0*L/u1) + pi*2*(u0<0);
    dVal = cat(1,dVal, d);
    xdot0 = (u1*tan(d))/L;
    xdot1 = u1*sin(theta);
    xdot2 = u1*cos(theta);

%     Dubins Model
%     xdot0 = u0;
%     xdot1 = u1*sin(frontAngle);
%     xdot2 = u1*cos(frontAngle);


    % Output
    xdot = [xdot0; xdot1; xdot2];
end