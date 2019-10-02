function inside = D(z) 
%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @  Hybrid Systems Laboratory (HSL), 
% https://hybrid.soe.ucsc.edu/software
% http://hybridsimulator.wordpress.com/
% Filename: D.m
%--------------------------------------------------------------------------
% Description: Jump set
% Return 0 if outside of D, and 1 if inside D
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   See also HYEQSOLVER, PLOTARC, PLOTARC3, PLOTFLOWS, PLOTHARC,
%   PLOTHARCCOLOR, PLOTHARCCOLOR3D, PLOTHYBRIDARC, PLOTJUMPS.
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.3 Date: 05/20/2015 3:42:00

    % Grab Global Variables
    global square target mu lambda
    % Grab state q
    x = z(1);
    y = z(2);
    q = z(3);
    distq = distFind(z,square,q);
    distq = distq(1);
    distq1 = distFind(z,square,1-q);
    distq1 = distq1(1);
    % If current V is <= other V then flow
    if V(z,target,distq) >= (mu - lambda)*V(z,target,distq1)
%         disp('Enter Jump')
        inside = 1;
    else
        inside = 0;
    end

end