function y = NMPC(x, kimp, params)
    dt = params.dt;
%     timp = heelstrike(x(1:4),params); % Find when heelstrike occurs
%     kimp = round(timp/dt);

    Qy = params.Qy;
    Qu = 1*eye(kimp);
    yref = params.yref;

    fun = @(u)fminconfun(u, x, Qy, Qu, kimp, yref, params);
    lb = -5*ones(kimp,1);
    ub = 0*ones(kimp,1);
    uInit = zeros(kimp,1);
%     opts = optimset('fmincon');
%     opts.Algorithm = 'sqp';
    opts = optimoptions('fmincon','Algorithm','sqp',...
    'OptimalityTolerance',1e-5);
%     options = optimoptions('fmincon','MaxFunctionEvaluations',5000);
    uopt = fmincon(fun,uInit,[],[],[],[],lb,ub,[],opts);
    params.mH; 
    y = uopt(1);
end


    