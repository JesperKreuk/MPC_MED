% This function works only for the right swing phase, similar can be done
% for the left swing phase, but in this case the model is symmetric.
function [Rdatax, Rdatau, Ndata] = calculateSensorData(LAnklePosxy,...
                RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,indices)
    L = 1;
    dt_visual = 1.0000e-03;
            
    RfootPosxy = RAnklePosxy;
    LfootPosxy = LAnklePosxy;
    HipPosxy = (LHipPosxy + RHipPosxy)/2-hipCorrection;

    
    % Theta 1 is always the left (healthy) leg 
    % Theta 2 is always the right (prosthetic) leg
    th1 = asin((HipPosxy(:,1)-LfootPosxy(:,1))/L);
    th2 = asin((HipPosxy(:,1)-RfootPosxy(:,1))/L);

    % Calculate dth1 and dth2
    dth1 = gradient(th1,dt_visual);
    dth2 = gradient(th2,dt_visual);

    % Define x and u
    x = [th1,th2,dth1,dth2];
    u = cos(x(:,1:2));

    % Cut data
    Rdatax = x(indices,:);
    Rdatau = u(indices,:);
    Ndata = length(indices);
end
