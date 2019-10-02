clc
clear all

addpath('../RegionObstacle')
% Run Hybrid Point Mass Model for trajectory
x0 = [-1; -1; 0];
goal = [3; 0];
obstacle = [1; -0.25; 1/(20*sqrt(2))];
findPath

% Delete hybrid q column
x(:,3) = [];

partial = [t x];
full_Orig = generateTrajectoryData(partial);

partial_2 = Extend_Ref(partial, 2);
full = generateTrajectoryData(partial_2);


% Plot Reference Path
figure(1)
plot(full_Orig(:,2),full_Orig(:,3))
hold on
plot(full(:,2),full(:,3))
title('Path')
legend('Original', 'New')
grid on

figure(2)
plot(full_Orig(:,1),full_Orig(:,4))
hold on
plot(full(:,1), full(:,4))
title('Theta')
legend('Original', 'New')

grid on

figure(3)
plot(full_Orig(:,1),full_Orig(:,5))
hold on
plot(full(:,1), full(:,5))
title('Linear Velocity')
legend('Original', 'New')

grid on

figure(4)
plot(full_Orig(:,1),full_Orig(:,6))
hold on
plot(full(:,1), full(:,6))
title('Angular Velocity')
legend('Original', 'New')
grid on

