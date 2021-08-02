%{
This function is called by the NMPC controller, it calculates the cost
function J1 of the corresponding thesis. For this it uses the RK4
integration method to find the future states.

Arguments:
* u: the input sequence containing the torques of the MED 
    u = [u(0),..., u(N-1)]
* x0: initial state of the CGB [th1;th2;dth1;dth2]
* Qy: Weight matrix that penalises the states
* Qu: Weight matrix that penalises the input
* x0: State of the current time step [th1;th2;dth1;dth2]
* N: Prediction horizon (integer)
* yref: Reference, this is the ankle to ankle distance in m
* params: structure containing the parameters of the CGB model

Output: 
* fx: function value of J1
* yend: the final ankle to ankle distance of the CGB simulation 

Author: Jesper Kreuk
%}

function [J1, yend] = NMPCcostFunction(u, x0, Qy, Qu, N, yref, params)
    % Extract the parameters of the CGB from the structure
    a = params.a;
    L = params.L;
    b = params.b;
    m = params.m;
    mH = params.mH;
    g = params.g;
    phi1 = params.phi1;
    phi2 = params.phi2;
    dt = params.dt;
   
    % Initialize integration of the state
    x = zeros(4,N); % [x(1), ... x(kimp)]
    xk = x0;
    for i = 1:N
        if xk(1) + xk(2) >= 0
            % If heel strike has occured, the state no longer changes
            x(:,i) = xk;
        else
            % Else RK4 integration with the nonlinear dynamics of the CGB
            % model
            k1 = dt*dynamicsNMPC(xk,u(i),m,a,mH,phi1,phi2);
            k2 = dt*dynamicsNMPC(xk+k1/2,u(i),m,a,mH,phi1,phi2);
            k3 = dt*dynamicsNMPC(xk+k2/2,u(i),m,a,mH,phi1,phi2);
            k4 = dt*dynamicsNMPC(xk+k3,u(i),m,a,mH,phi1,phi2);
            x(:,i) = xk + (k1+2*k2+2*k3+k4)/6;
        end
        xk = x(:,i);
    end
    % Select final state
    xend = x(:,end);
    yend = sin(xend(1))-sin(xend(2));
    
    % Calculate value of cost function J1
    J1 = (yend-yref)'*Qy*(yend-yref) + u'*Qu*u;
end




