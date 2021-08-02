%{
This function can make a single simulation steps given the MLD system.

Arguments:
* S: Structure containing the MLD model
* x0: current state of the CGB model [th1;th2;dth1;dth2]
* u: input vector containing the torques of the MED

Output: 
* x1: The next state 
* y1: The next output (ankle to ankle distance)
* flag: logical boolean, 1 if an MLD constraint is violated, 0 else
* w0: A vector containing the auxiliary variables

Author: Jesper Kreuk
%}
function [x1, y1, flag, w0] = onestepMLD(S,x0,u0)
    % First calculate all auxiliary variables w
    c = 0.001; % only able to switch if th1 >= c
    z1 = x0(1:4)*x0(5);
    z2 = u0*x0(5);
    if x0(1) >= c % check if in late swing
        deltaLS = 1;
    else
        deltaLS = 0;
    end
    if x0(1)+x0(2) >= 0 % check if an impact has occured
        deltaimp = 1;
    else
        deltaimp = 0;
    end
    if deltaimp+deltaLS+x0(5) >= 3 % if in late swing and an impact has occured and state is still in prosthetic swing, switch
        deltaSW = 1;
    else
        deltaSW = 0;
    end
    w0 = [z1;z2;deltaLS; deltaimp;deltaSW]; % The auxiliary variables
    
    % Dynamical equations
    x1 = S.A*x0+S.Bu*u0+S.Baux*w0+S.Baff;
    y1 = S.C*x1;
    
    % Check if any constraints are violated
    check_conditions = S.Ex*x0+S.Eu*u0+S.Eaux*w0 <= S.Eaff;
    flag = any(~check_conditions); % flag = 1 is a violation of constraints
end