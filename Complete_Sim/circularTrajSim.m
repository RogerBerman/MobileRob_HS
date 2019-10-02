clc
clear all

addpath('ODE_Solvers')
addpath('Region_Obstacle')
addpath('Control_Funcs')
addpath('Trajectory_Gen')
addpath('ToolBox_Fitting_Funcs')

%% Create Circular path
path = circle(0,0,1.5);
path_len = length(path);
step = 0.3;
time = 0:step:step*(path_len-1);
path = [time' path];

%% Create Trajectory
% scale = 15;
% path_ext = Extend_Ref(path,1,scale);
trajTime = time(end);
ref = generateTrajectoryData(path,1);
circRef = ref;
% GenPlotTraj

%% Travel Trajectory
% Grab u0,u1 values
global input0 input1 dVal L counter eX eY eTheta 
input0 = [];
input1 = [];
L = 0.35;
counter = 0;
eX = [];
eY = [];
eTheta = [];

dVal = [];
dVal = cat(1,dVal,0);

% start location
x0 = [pi/2 0 1.75];
x = x0;
THETA = [];
Y = [];
X = [];
TIME = [];

% Init time and control loop speed
sysTime = 0;
control_time = step;
% Solve ODE at each timestep
for index = 1:control_time:trajTime
    start = [x(end,1) x(end,2) x(end,3)];
    t = [sysTime sysTime+control_time];
    circRef = locateTraj(circRef,sysTime);
    x = ode2(@(t,x) sys(t,x,circRef(1,2:6)), t , start);
    
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


%% Plotting code

% Plot Reference Path
figure()
% Plot Marker on Inital Position
plot(x0(3),x0(2), '^');
hold on
% Plot Reference Path
plot(ref(:,2), ref(:,3))
hold on
% Plot Vehicle Path
plot(X,Y,'o')
title('Position')
xlabel('X Coordinate (m)')
ylabel('Y Coordinate (m)')
legend('Start','Reference', 'Vehicle')
xlim([-2 2])
ylim([-2 2])
grid on

figure()
plot(0:TIME(end-1)/(length(input0)-1):TIME(end-1),input0)
hold on
plot(ref(:,1), ref(:,6))
xlabel('Time (s)')
ylabel('Angular Velocity (r/s)')
title('Angular Velocity')
legend('Input', 'Reference')
grid on
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

figure()
plot(0:TIME(end-1)/(length(eTheta)-1):TIME(end-1),eTheta)
xlabel('Time (s)')
ylabel('\theta Error (r)')
title('Heading Error')
grid on

figure()
plot(0:TIME(end-1)/(length(eX)-1):TIME(end-1),eX)
xlabel('Time (s)')
ylabel('X Coordinate Error (m)')
title('X Position Error')
grid on

figure()
plot(0:TIME(end-1)/(length(eY)-1):TIME(end-1),eY)
xlabel('Time (s)')
ylabel('Y Coordinate Error (m)')
title('Y Position Error')
grid on

%% Functions
function  xdot = sys(t, x, ref)
    global input0 input1 dVal L counter eX eY eTheta
    %set Values
    
    XR = ref(1);
    YR = ref(2);
    ThetaR = ref(3);
    vR = ref(4);
    wR = ref(5);
    
    % Calculate Error
    [X0,X1,X2] = errorCalc(ThetaR,YR,XR,x);
    eX = cat(1,eX,X2);
    eY = cat(1,eY,X1);
    eTheta = cat(1,eTheta,X0);
    % Input Error to Controller
    [u0,u1] = controllerCircTraj(X0,X1,X2,t,wR,vR);
    % Collect the results:
    input0 = cat(1,input0, u0);
    input1 = cat(1,input1, u1);

    % Dubins Model Differential Equations
    theta = x(1);
    %     Bicycle Model
%     d = atan2(u0*L,u1) + pi*2*(u0<0) + pi*2*counter;
    % deal with roll over degrees
%     if dVal(end)-d > pi
%         counter = counter + 1;
%         d = atan2(u0*L,u1) + pi*2*(u0<0) + pi*2*counter;
%     end

    d = atan2(u0*L,u1);
    dVal = cat(1,dVal, d);
    d_tan = tan(d);
    xdot0 = (u1*d_tan)/L;
    xdot1 = u1*sin(theta);
    xdot2 = u1*cos(theta);
    
    
%     Dubins Model
%     xdot0 = u0;
%     xdot1 = u1*sin(theta);
%     xdot2 = u1*cos(theta);


    % Output
    xdot = [xdot0; xdot1; xdot2];
end

function [path] = circle(x,y,r)
    th = 0:pi/50:3*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    path = [xunit' yunit'];
end