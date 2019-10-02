% Description: This function serves to transmit the current ppmValues to 
% the vehicle and identify if the vehicle is outside the designated 
% trackable field. If the vehicle is reaches the limit of this field it 
% is stoppped.
% Input: 
%     x coordinate of vehicle, 
%     y coordinate of vehicle,
%     ppmValues of vehicle
% Output: None
function [] = safeTransmit(s, ppmValues, x, y)
    sendPPM = ppmValues;
    % If any condition below is satisfied stop vehicle
    if x > 3.25 || x < -3
        str2 = num2str(x);
        str1 = 'X Axis Limit Reached: ';
        s = strcat(str1,str2);
        disp(s);
        sendPPM = [1500,1500,1500,1500,1500,1500];
    end
    if y > 1.5 || y < -1.4
        str2 = num2str(y);
        str1 = 'Y Axis Limit Reached: ';
        s = strcat(str1,str2);
        disp(s);
        sendPPM = [1500,1500,1500,1500,1500,1500];
    end
    % Send new PPM values to Vehicle
    transmitSerial(s, sendPPM);

end