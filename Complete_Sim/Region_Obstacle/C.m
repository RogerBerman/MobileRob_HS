function [value] = C(z) 
%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @  Hybrid Systems Laboratory (HSL), 
% https://hybrid.soe.ucsc.edu/software
% http://hybridsimulator.wordpress.com/
% Filename: C.m
%--------------------------------------------------------------------------
% Description: Flow set
% Return 0 if outside of C, and 1 if inside C
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   See also HYEQSOLVER, PLOTARC, PLOTARC3, PLOTFLOWS, PLOTHARC,
%   PLOTHARCCOLOR, PLOTHARCCOLOR3D, PLOTHYBRIDARC, PLOTJUMPS.
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.3 Date: 05/20/2015 3:42:00

    % Global Variables 
    global square target mu
    % Grab State q
    x = z(1);
    y = z(2);
    q = z(3);
    % If current V is <= other V then flow
    distq = distFind(z,square,q);
    distq = distq(1);
    distq1 = distFind(z,square,1-q);
    distq1 = distq1(1);
    if V(z,target,distq) <= mu*V(z,target,distq1)
        value = 1;
    else
%         disp('Leave Flow')
        value = 0;
    end
%     value = 1;

end