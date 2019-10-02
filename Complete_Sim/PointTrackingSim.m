clc
clear all

addpath('ODE_Solvers')
addpath('Region_Obstacle')
addpath('Control_Funcs')
addpath('Trajectory_Gen')
addpath('ToolBox_Fitting_Funcs')


%% Solve ODE at fixed timesteps based on Reference Trajectory

% Grab u0,u1 values
global input0 input1 dVal L
input0 = [];
input1 = [];
L = 0.35; 

dVal = [];

% start location
x = [0 10 10];
THETA = [];
Y = [];
X = [];
TIME = [];

% Init time and control loop speed
sysTime = 0;
control_time = 0.05;
% Solve ODE at each timestep
while(1)
    start = [x(end,1) x(end,2) x(end,3)];
    dist = sqrt(start(2)^2 + start(3)^2);
    if dist < 0.15
        break;
    end
    t = [sysTime sysTime+control_time];
    x = ode2(@(t,x) sys(t,x), t , start);
    
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

figure(5)
% Plot Marker on Inital Position
plot(10,10, '^');
hold on
% Plot Marker on Target
plot(0,0, 'x');
hold on
% Plot Vehicle Path
plot(X,Y)
title('Position')
xlabel('X Coordinate(m)')
ylabel('Y Coordinate(m)')
legend('Start', 'Goal', 'Vehicle')
xlim([-1 12])
ylim([-1 11])
grid on

figure(6)
plot(TIME,rad2deg(THETA))
title('Theta vs Time')
ylabel('Theta(deg)')
xlabel('Time(s)')
grid on

figure(7)
plot(TIME,Y)
title('Y vs Time')
ylabel('Y Coordinates(m)')
xlabel('Time(s)')
grid on

figure(8)
plot(TIME,X)
title('X vs Time')
ylabel('X Coordinates(m)')
xlabel('Time(s)')
grid on

figure(9)
plot(0:TIME(end-1)/(length(input0)-1):TIME(end-1),input0)
title('U0 vs Time')
ylabel('U0 (Angular Velocity(deg))')
xlabel('Time()')
grid on

figure(10)
plot(0:TIME(end-1)/(length(input1)-1):TIME(end-1),input1)
title('U1 vs Time')
ylabel('U1 (Linear Velocity(m/s))')
xlabel('Time')
grid on

figure(11)
plot(0:TIME(end-1)/(length(dVal)-1):TIME(end-1),rad2deg(dVal))
title('Delta vs Time')
ylabel('Delta(deg)')
xlabel('Time(s)')
grid on

%% Functions
function  xdot = sys(t, x)
    global input0 input1 dVal L 
    %set Values
    
    XR = 0;
    YR = 0;
    ThetaR = 0;
    vR = 0;
    wR = 0;
    % Calculate Error
    [X0,X1,X2] = errorCalc(ThetaR,YR,XR,x);
    % Input Error to Controller
    [u0,u1] = controllerPointTracking(X0,X1,X2,t,wR,vR);
    % Collect the results:
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
    d = atan(u0*L/u1);
    dVal = cat(1,dVal, d);
    xdot0 = (u1*tan(d))/L;
    xdot1 = u1*sin(theta);
    xdot2 = u1*cos(theta);

    % Output
    xdot = [xdot0; xdot1; xdot2];
end