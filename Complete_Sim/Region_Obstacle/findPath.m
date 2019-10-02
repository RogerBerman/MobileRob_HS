%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @  Hybrid Systems Laboratory (HSL), 
% https://hybrid.soe.ucsc.edu/software
% http://hybridsimulator.wordpress.com/
% Filename: findPath.m
%--------------------------------------------------------------------------
% Project: Find steepest descent path
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
global target obs initial region square mu lambda 
% initial conditions                                                    
% x0 = [-0.5; -0.25; 1];
% goal = [3; 0];
% % Obstacle [X2 X1 R]
% obstacle = [1 0 1/(20*sqrt(2))];
% simTime = 5;
% Square points around Obstacle 
lp = [obstacle(1)-obstacle(3), obstacle(2)];
rp = [obstacle(1)+obstacle(3), obstacle(2)];
bp = [obstacle(1), obstacle(2)-obstacle(3)];
tp = [obstacle(1), obstacle(2)+obstacle(3)];
% Square around obstacle
square = [lp;rp;tp;bp];
                                                                                              
target = goal;
obs = obstacle;
initial = x0;
region = x0(3);
mu = 1.1;
lambda = 0.09;

% simulation horizon                                                    
T = simTime;                                                                 
J = 5;                                                                 

% rule for jumps                                                        
% rule = 1 -> priority for jumps                                        
% rule = 2 -> priority for flows                                        
% rule = 3 -> no priority, random selection when simultaneous conditions
rule = 2;                                                               

%solver tolerances
RelTol = 1e-6;
MaxStep = 1e-3;

sim('ObstacleSimTC',simTime)
% postprocessing