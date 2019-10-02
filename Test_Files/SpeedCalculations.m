% Speed Program
%% Velocity Function
Velocity = @(init,final,interval) sqrt((final(1)-init(1))^2 + (final(2)-init(2))^2)/interval;

%% Velocity at 1630 ppm
% Coordinates [X; Y] (in Meters)
init = [-1.914; -0.7626];
final = [0.0243; -0.5633];
% Time = Final - Initial (in Seconds)
timeStep = 3.7651 - 0.0017;

Velocity_PPM1630 = Velocity(init,final, timeStep);

%% Velocity at 1640 ppm
% Coordinates [X; Y] (in Meters)
init = [-2.0353; -0.5727];
final = [0.3203; -0.4209];
% Time = Final - Initial (in Seconds)
time = 3.7649 - 0.0018;

Velocity_PPM1640 = Velocity(init,final, time);

%% Velocity at 1650 ppm
% Coordinates [X; Y] (in Meters)
init = [-2.4810; -0.5278];
final = [0.2479; -0.1939];
% Time = Final - Initial (in Seconds)
time = 3.7642 - 0.0021;

Velocity_PPM1650 = Velocity(init,final, time);

%% Velocity at 1660 ppm
% Coordinates [X; Y] (in Meters)
init = [-2.987; -0.2448];
final = [0.4706; -0.5818];
% Time = Final - Initial (in Seconds)
time = 3.7674 - 0.0043;

Velocity_PPM1660 = Velocity(init,final, time);

%% Velocity at 1670 ppm
% Coordinates [X; Y] (in Meters)
init = [-2.8393; -0.7018];
final = [1.0589; -0.3342];
% Time = Final - Initial (in Seconds)
time = 3.7650 - 0.0032;

Velocity_PPM1670 = Velocity(init,final, time);

%% Velocity at 1680 ppm
% Coordinates [X; Y] (in Meters)
init = [-3.0658; -0.4505];
final = [1.0321; -0.0414];
% Time = Final - Initial (in Seconds)
time = 3.7702 - 0.0017;

Velocity_PPM1680 = Velocity(init,final, time);

%% Velocity at 1690 ppm
% Coordinates [X; Y] (in Meters)
init = [-2.9607; -0.4993];
final = [1.16; -0.3362];
% Time = Final - Initial (in Seconds)
time = 3.7634 - 0.0023;

Velocity_PPM1690 = Velocity(init,final, time);

%% Velocity at 1700 ppm
% Coordinates [X; Y] (in Meters)
init = [-3.1641; -0.3678];
final = [1.1715; -0.4192];
% Time = Final - Initial (in Seconds)
time = 3.7643 - 0.0044;

Velocity_PPM1700 = Velocity(init,final, time);


Velocities = [Velocity_PPM1630 Velocity_PPM1640 Velocity_PPM1650 Velocity_PPM1660...
 Velocity_PPM1670 Velocity_PPM1680 Velocity_PPM1690 Velocity_PPM1700];

PPM = 1630:10:1700;

Linza = scatter(Velocities, PPM);
h = lsline;
grid on

m = (h.YData(2)-h.YData(1))/(h.XData(2) - h.XData(1))
b = h.YData(1)-m*h.XData(1)

y = round(m*(0.75)+b)
