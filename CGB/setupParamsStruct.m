function params = setupParamsStruct(parametersOpt, dt, controlParams)
    if nargin == 2
        params.dt = dt;
    elseif nargin == 3
        params.dt = dt;
        params.Qy = controlParams.Qy;
        params.yref = controlParams.yref;
    end
    m = parametersOpt(1);
    a =  parametersOpt(2);
    mH =  parametersOpt(3);
    phi1 =  parametersOpt(4);
    if length(parametersOpt)>4
        phi2 =  parametersOpt(5);
    else
        phi2 = phi1;
    end
    L = 1;
    b = L-a;
    g = 9.81;
    params.a = a;
    params.L = L;
    params.b = b;
    params.m = m;
    params.mH = mH;
    params.g = g;
    params.phi1 = phi1;
    params.phi2 = phi2;
    
    params.hipCorrection = 0.0534;
end