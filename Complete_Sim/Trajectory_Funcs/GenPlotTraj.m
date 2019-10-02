%% Plot Generated Trajectory

% Plot Reference Path
% figure(1)
% plot(refU(:,2),refU(:,3))
% title('Original Path')
% xlabel('X Coordinate (m)')
% ylabel('Y Coordinate (m)')
% grid on
% 
% figure(2)
% plot(refU(:,1), refU(:,4))
% title('Original Theta vs Time')
% ylabel('Theta (deg)')
% xlabel('Time (s)')
% grid on
% 
% figure(3)
% plot(refU(:,1), refU(:,5))
% title('Original Linear Velocity vs Time')
% ylabel('Linear Velocity (m/s)')
% xlabel('Time (s)')
% grid on
% 
% figure(4)
% plot(refU(:,1), refU(:,6))
% title('Original Angular Velocity vs Time')
% ylabel('Angular Velocity (deg/s)')
% xlabel('Time (s)')
% grid on

% Plot Reference Path
figure(5)
plot(ref(:,2),ref(:,3))
title('Final Path')
xlabel('X Coordinate (m)')
ylabel('Y Coordinate (m)')
grid on

figure(6)
plot(ref(:,1), ref(:,4))
title('Final Theta vs Time')
ylabel('Theta (deg)')
xlabel('Time (s)')
grid on

figure(7)
plot(ref(:,1), ref(:,5))
title('Final Linear Velocity vs Time')
ylabel('Linear Velocity (m/s)')
xlabel('Time (s)')
grid on

figure(8)
plot(ref(:,1), ref(:,6))
title('Final Angular Velocity vs Time')
ylabel('Angular Velocity (deg/s)')
xlabel('Time (s)')
grid on