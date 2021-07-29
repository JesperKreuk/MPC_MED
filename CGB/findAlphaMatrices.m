function [J3Data, J3Model] = findAlphaMatrices(dataset, swingNumber, params)
    load(dataset,'simout');
    loadGreyboxPositions
    
    swingIndices = findSwingIndices(dataset,swingNumber);
    hipCorrection = calculateHipCorrection(dataset,10);
    %% Extract data from swingnumber (x,u)
    Rdatax = calculateSensorData(LAnklePosxy,...
                RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,swingIndices);
    %% Find the optimal alpha based on RMSE
    
    m = params.m;
    a =  params.a;
    mH =  params.mH;
    phi1 =  params.phi1;
    phi2 = params.phi2;
    
    dt = 1e-2;
    RdataxRes = Rdatax(1:10:end-5,:).';
    x0 = RdataxRes(:,1);
    timp = heelstrike(x0,params); % Find when heelstrike occurs
    kimp = round(timp/dt);



    alphaRange = 0:0.01:1;
    cost_func = 'NRMSE';

    for k = 1:kimp
        for j = 1:length(alphaRange) 
            alpha = alphaRange(j);
            x0 = RdataxRes(:,k);

            tspan = 0:dt:0.6;
            [~,xnonlin] = ode45(@(t,x) dynamics(t,x,m,a,mH,phi1,phi2), tspan,x0);

            sw_cond = xnonlin(:,1)+xnonlin(:,2); % Switch condition th1+th2 = 0
            crossIdx = find(sw_cond(2:end).*sw_cond(1:end-1)<0);

            if isempty(crossIdx)
                crossIdx = 1;
            end
            crossIdx = crossIdx(end);
            % The linearization index at alpha = 0.5
            xend = xnonlin(crossIdx,:).';
            xeq = x0*(1-alpha) + xend*alpha;
            A = Asymfun(xeq,0);
            Baff = Baffsymfun(xeq,0);
            xlin = x0;
            for i = 1:kimp-k
                xdot(:,i) = A*xlin(:,i)+Baff;
                xlin(:,i+1) = xlin(:,i)+xdot(:,i)*dt;
            end

            J3Data(j,k) = sum(goodnessOfFit([xlin(1,:);xlin(2,:)].',[RdataxRes(1,k:end);RdataxRes(2,k:end)].',cost_func));
            J3Model(j,k) = sum(goodnessOfFit([xlin(1,:);xlin(2,:)].',[xnonlin(1:kimp-k+1,1),xnonlin(1:kimp-k+1,2)],cost_func));
        end
    end
end


