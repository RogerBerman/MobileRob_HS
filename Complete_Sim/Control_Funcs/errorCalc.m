function [X0,X1,X2] = errorCalc(ThetaR,YR,XR,cur)
%   Variable Assignmnet
    thetaR = ThetaR;
    theta = cur(1);
    yR = YR;
    y = cur(2);
    xR = XR;
    x = cur(3);
    
%     Calculation
    X0 = thetaR - theta;
    X1 = sin(theta)*(x - xR) - cos(theta)*(y - yR);
    X2 = -1*(-cos(theta)*(x - xR) - sin(theta)*(y - yR));
end