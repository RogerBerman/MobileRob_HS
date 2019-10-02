
addpath('../Complete_Sim')
% start location
x = [0 0 0];
THETA = [];
X = [];
Y = [];

% Sim Time
time = 0:0.1:10;
timeLen = length(time);
% System of equations
for index = 1:1:timeLen-1
    start = [x(end,1) x(end,2) x(end,3)];
    t = [time(index) time(index+1)];
    x = ode2(@(t,x) sys(x,0.005,0.5), t , start);
    
    % Collect the results:
    THETA = cat(1,THETA, x(end,1));
    X = cat(1,X, x(end,2));
    Y = cat(1,Y, x(end,3));

end

% Plotting Code
x= [THETA X Y];
t = time;

figure(1)
hold on
plot(x(:,2),x(:,3))
title('Simulation Position')
xlabel('X Coordinates')
ylabel('Y Coordinates')
grid on


%% Functions
function  xdot = sys(x,Ang,Lin)

    % Variables
    theta = x(1);
    u0 = Ang;
    u1 = Lin;
    
    
    %     Bicycle Model
    d = atan(u0*0.1/u1);
    xdot0 = (u1*tan(d))/0.1;
    xdot1 = u1*sin(theta);
    xdot2 = u1*cos(theta);

    % Output
    xdot = [xdot0; xdot1; xdot2];
end