%  Controller
% Inputs: reference matrix, current position
% Outputs: linear velocity, angular velocity (u2,u1)

% ref: [Time X1 X2 LinearVelocity Theta AngularVelocity]
% cur: [X1 X2 Theta]
function [u0, u1] = controllerPhysical(x0,x1,x2,t,wR,vR)
    
    % Constants
    % k0 > 0
    % k1 > 0
    % 0 < alpha < vmax - sup(abs(vref))
    % b > 0
    % 0 < gamma < 1
    % 0 < epsilon < 1/(1+gamma)
    
%%     Bicycle Model Parameters

    % CIRCLE PARAMETERS
    % Linear
    k0 = 2.28;
    a = 1.3;
    %Angular
    k1 = 0.219;
    b = 0.3;
    gamma = 0.5;
    epsilon = 0.5;
    mu = 1;
    
%     % O0 Flow PARAMETERS
%     % Linear
%     k0 = 1.9;
%     a = 1.3;
%     %Angular
%     k1 = 0.5;
%     b = 0.3;
%     gamma = 0.5;
%     epsilon = 0.5;
%     mu = 1;
    
%     % O1 Flow PARAMETERS
%     % Linear
%     k0 = 1.55;
%     a = 1.3;
%     %Angular
%     k1 = 0.5;
%     b = 0.3;
%     gamma = 0.5;
%     epsilon = 0.5;
%     mu = 1;
    
%     % O1/O0 Jump PARAMETERS
%     % Linear
%     k0 = 1.55;
%     a = 1.3;
%     %Angular
%     k1 = 0.55;
%     b = 0.3;
%     gamma = 0.5;
%     epsilon = 0.5;
%     mu = 1;


    % STRAIGHT PARAMETERS
     % Linear
%     k0 = 0.5;
%     a = 1.3;
%     %Angular
%     k1 = 1.5;
%     b = 0.3;
%     gamma = 0.5;
%     epsilon = 0.5;
%     mu = 1;
    
%% Control Laws
    % linear Velocity output
    u1 = double(-1*sat(k0*x2, a));
    
    % Find angular Velocity output
    
    % Variable required for Alpha Function
    h = 1 + gamma*cos(mu*t);
    V1 = x1^2 + x2^2;
    % Alpha function
    alphaFunc = 1 - ((epsilon*h*x2)/(1 + sqrt(V1)));
    
    % Variables required for Beta Function
    dh = -mu*gamma*sin(mu*t);

    % Pieces of Beta Function
    part1 = (dh*x1 + h*wR*x2 + h*vR*sin(x0))/(1 + sqrt(V1));
    %ISSUE HERE WHEN SAME STARTING POINT AS REF PATH
    part2 = (h*x1)/( ((1 + sqrt(V1))^2) * sqrt(V1) );
    
    part3 = (x1*vR*sin(x0) - sat(k0*x2,a)*x2);
    betaFunc = epsilon * (part1 - part2 * part3);
    
    % Variables required for Angular Velocity Output
    Nx0 = x0 + (epsilon*h*x1)/(1 + sqrt(V1));
    
    % Angular Velocity Output
    u0 = double(((betaFunc)/alphaFunc) + sat(k1*Nx0,b));
    
    % Stabilization Guards
    if abs(u0) < 0.005
        u0 = 0;
    end
    if u1 < 0.001
        u1 = 0.001;
    end
    % Error Model 2
%     vOut = u1 + vR*cos(x0);
%     wOut = wR - u0;
%     
%     u0 = wOut;
%     u1 = vOut;

end