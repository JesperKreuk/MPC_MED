function [timp, ximp, CP] = heelstrike(x0, params)
    % The nonlinear model of the compass walker is actuated by the virtual
    % gravity only. The moment of impact is determined and the variables
    % are extracted
    %
    % Inputs
    % * x0: current state
    % * params: struct containing all relevant parameters
    % Outputs
    % * timp: time it takes before impact (in seconds)
    % * ximp: the state at the moment of impact 
    % * CP: the capture point
    
    a = params.a;       % Distance from heel to CoM leg
    L = params.L;       % Leg length
    b = params.b;       % Distance from CoM leg to hip
    m = params.m;       % Mass of the leg
    mH = params.mH;     % Mass of the hip
    g = params.g;       % Gravitational acceleration
    phi1 = params.phi1; % Virtual gravity angle stance leg
    phi2 = params.phi2; % Virtual gravity angle swing leg  
    
    omega0 = sqrt(g/L); % Constant for calculating the capture point
    
    % Simulate the nonlinear model 
    tspan = 0:1e-3:0.55;
    [t,xnonlin] = ode45(@(t,x) CWodefun(t,x,m,a,mH,phi1,phi2), tspan, x0);

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
    
    % The capture point when the model hits the ground
    th1 = ximp(1);
    th2 = ximp(2);
    dth1 = ximp(3);
    dth2 = ximp(4);
    xCoM = (m*a*sin(th1)+mH*L*sin(th1)+m*(L*sin(th1)+b*sin(-th2)))/(2*m+mH);
    dxCoM = (m*a*cos(th1)*dth1+mH*L*cos(th1)*dth1+m*(L*cos(th1)*dth1-b*cos(-th2)*dth2))/(2*m+mH);
    CP = xCoM+dxCoM/omega0;
end