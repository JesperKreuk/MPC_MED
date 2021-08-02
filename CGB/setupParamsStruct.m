%{
This function sets up the paremeter structure containing the parameters of 
the CGB model, it is optional to add control parameters as well.

Arguments:
* parametersOpt: The optimized parameters [m, a, mH, phi1, phi2] of the CGB
    model
* dt: step time of the model
* controlParams: structure that contains Qy and yref for control (weight
    and ankle to ankle reference)

Output: 
* params: a structure containing the parameters of the CGB

Author: Jesper Kreuk
%}

function params = setupParamsStruct(parametersOpt, dt, controlParams)
    if nargin == 2
        % if there are 2 arguments specify time
        params.dt = dt;
    elseif nargin == 3
        % if there are 3 arguments specify time and control parameters
        params.dt = dt;
        params.Qy = controlParams.Qy;
        params.yref = controlParams.yref;
    end
    
    % Set the optimized parameters 
    m = parametersOpt(1);
    a =  parametersOpt(2);
    mH =  parametersOpt(3);
    phi1 =  parametersOpt(4);
    if length(parametersOpt)>4
        phi2 =  parametersOpt(5);
    else
        phi2 = phi1;
    end
    
    % Define other constants
    L = 1;      % leg length in m
    b = L-a;    % distance CoM of leg to hip in m
    g = 9.81;   % gravitational acceleration in m/s/s
    
    % Setup the params structure
    params.a = a;
    params.L = L;
    params.b = b;
    params.m = m;
    params.mH = mH;
    params.g = g;
    params.phi1 = phi1;
    params.phi2 = phi2;
    
    % The hip correction if the ankle positions are tracked, see function
    % findHipCorrection
    params.hipCorrection = 0.0534;
end