function y = NMPC(x, kimp, params)
    Qy = params.Qy;
    Qu = 1*eye(kimp);
    yref = params.yref;

    fun = @(u)NMPCcostFunction(u, x, Qy, Qu, kimp, yref, params);
    lb = -5*ones(kimp,1);
    ub = 0*ones(kimp,1);
    uInit = zeros(kimp,1);
    opts = optimoptions('fmincon','Algorithm','sqp',...
    'OptimalityTolerance',1e-5);
    uopt = fmincon(fun,uInit,[],[],[],[],lb,ub,[],opts);
    params.mH; 
    y = uopt(1);
end


    