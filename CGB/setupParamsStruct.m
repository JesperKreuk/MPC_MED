function params = setupParamsStruct(ParametersOpt, Qy, yref, dt)
    m = ParametersOpt(1);
    a =  ParametersOpt(2);
    mH =  ParametersOpt(3);
    phi1 =  ParametersOpt(4);
    if length(ParametersOpt)>4
        phi2 =  ParametersOpt(5);
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
    params.dt = dt;
%     params.hipCorrection = 0.0816;
    params.hipCorrection = 0.0534;

    params.Qy = Qy;
    params.yref = yref;
end