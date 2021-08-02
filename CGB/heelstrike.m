%{
This function calculates when heelstrike occurs

Arguments:
* x0: the current state of the CGB model [th1;th2;dth1;dth2]
* params: a structure containing the parameters of the CGB model

Output: 
* timp: The time in seconds from x0 that heel strike will occur
* ximp: the state at the moment heel strike occurs

Author: Jesper Kreuk
%}

function [timp, ximp] = heelstrike(x0, params)   
    a = params.a;       % Distance from heel to CoM leg
    L = params.L;       % Leg length
    b = params.b;       % Distance from CoM leg to hip
    m = params.m;       % Mass of the leg
    mH = params.mH;     % Mass of the hip
    g = params.g;       % Gravitational acceleration
    phi1 = params.phi1; % Virtual gravity angle stance leg
    phi2 = params.phi2; % Virtual gravity angle swing leg  
    
    
    % Simulate the nonlinear model 
    tspan = 0:1e-3:0.55;
    [t,xnonlin] = ode45(@(t,x) dynamics(t,x,m,a,mH,phi1,phi2), tspan, x0);

    % Find when the model hits the ground
    sw_cond = xnonlin(:,1)+xnonlin(:,2); % Switch condition th1+th2 = 0
    crossIdx = find(sw_cond(2:end).*sw_cond(1:end-1)<0);
    if isempty(crossIdx)
        crossIdx = 1;
    end
    
    % The time when the model hits the ground
    timp = t(crossIdx(end));
    
    % The state when the model hits the ground
    ximp = xnonlin(crossIdx(end),:).';
end