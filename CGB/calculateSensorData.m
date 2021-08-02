%{
This function can calculate the states of the CGB model, it will calculate
the angles and angular velocity of the CGB as well as the torques exerted
on the CGB by the virtual gravity vector. The angles are calculated from
data of a 3D NMS model.

Arguments:
* LFootPosxy: Matrix containing the x and y position of the left foot
* RFootPosxy: Matrix containing the x and y position of the right foot
* LHipPosxy: Matrix containing the x and y position of the left hip
* RHipPosxy: Matrix containing the x and y position of the right hip


Output: 
* Rdatax: Matrix containing the states [th1;th2;dth1;dth2] where th is
    short for theta
* Rdatau: Matrix containing the torques [uAnk; uHip] from virtual gravity
* Ndata: Number of data points of the swing
* hipCorrection: a constant offset in the px direction of the hip of the
    CGB model with respect to the hip of the NMS model
* swingIndices: All indices between toe off and heel strike of a certain 
    swing

Author: Jesper Kreuk
%}
function [Rdatax, Rdatau, Ndata] = calculateSensorData(LFootPosxy,...
                RFootPosxy,LHipPosxy,RHipPosxy,hipCorrection,swingIndices)
    L = 1;
    dt = 1e-3;
            
    RfootPosxy = RFootPosxy;
    LfootPosxy = LFootPosxy;
    HipPosxy = (LHipPosxy + RHipPosxy)/2-hipCorrection;

    
    % Theta 1 is always the left (healthy) leg 
    % Theta 2 is always the right (prosthetic) leg
    th1 = asin((HipPosxy(:,1)-LfootPosxy(:,1))/L);
    th2 = asin((HipPosxy(:,1)-RfootPosxy(:,1))/L);

    % Calculate dth1 and dth2
    dth1 = gradient(th1,dt);
    dth2 = gradient(th2,dt);

    % Define x and u
    x = [th1,th2,dth1,dth2];
    u = cos(x(:,1:2));

    % Cut data
    Rdatax = x(swingIndices,:);
    Rdatau = u(swingIndices,:);
    Ndata = length(swingIndices);
end
