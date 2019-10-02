%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @  Hybrid Systems Laboratory (HSL), 
% https://hybrid.soe.ucsc.edu/software
% http://hybridsimulator.wordpress.com/
% Filename: postprocessing_ex1_2a.m
%--------------------------------------------------------------------------
% Project: Simulation of a hybrid system (bouncing ball)
% Description: postprocessing for the bouncing ball example
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   See also HYEQSOLVER, PLOTARC, PLOTARC3, PLOTFLOWS, PLOTHARC,
%   PLOTHARCCOLOR, PLOTHARCCOLOR3D, PLOTHYBRIDARC, PLOTJUMPS.
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.3 Date: 05/20/2015 3:42:00

global target obs initial region
% plot solution
figure()
regionPrint(obs, region)
printODE(initial, x, target, obs)



function [] = printODE(ip1, p1, target, obs)
    % Plot Marker on Inital Position
    plot(ip1(1),ip1(2), '^');
    hold on
    % Plot Marker on Target
    plot(target(1),target(2), 'x');
    hold on
    % Obstacle
    plot(obs(1),obs(2), '.', 'MarkerSize',10);
    hold on
    % Obstacle
    %Bounds
    xlim([-2 4])
    ylim([-2 2])
    grid on
    %Plot Paths
    plot(p1(:,1), p1(:,2));

end

