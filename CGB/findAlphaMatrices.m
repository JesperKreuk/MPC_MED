%{
This function calculates how well a linearized model fits the data and the
nonlinear model. To achieve this a range of different linearization points
are tested by using a grid search.
The linearization point (xeq) is chosen as follows:
            xeq = x0*(1-alpha) + xend*alpha
where alpha is between 0 and 1, it is thus a convex combination of the two
piont. The optimal value of alpha is optimized.

Arguments:
* dataset: the dataset containing the positions required for the greybox
    estimation
* swingNumber: which swing of the data is used for greybox estimation
* params: a structure containing all parameters of the CGB model

Output: 
* J3Data: The value of the cost function J3 with the data of the 3D NMS 
    model as a reference
* J3Model:The value of the cost function J3 with the nonlinear CGB model 
    as a reference


Author: Jesper Kreuk
%}

function [J3Data, J3Model] = findAlphaMatrices(dataset, swingNumber, params)
    % Load data and positions
    load(dataset,'simout');
    loadGreyboxPositions
    
    % Find the indices corresponding to the swing and extract the hip
    % correction of identification set 10
    swingIndices = findSwingIndices(dataset,swingNumber);
    hipCorrection = calculateHipCorrection(dataset,10);
    
    % Calculate the states of the CGB model in the swing
    Rdatax = calculateSensorData(LAnklePosxy,...
                RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,swingIndices);

    % Extract parameters of the CGB model from the structure
    m = params.m;
    a =  params.a;
    mH =  params.mH;
    phi1 =  params.phi1;
    phi2 = params.phi2;
    
    % time step of discrete system
    dt = 1e-2;
    
    % Resample the state data 
    RdataxRes = Rdatax(1:10:end-5,:).';
    x0 = RdataxRes(:,1);
    timp = heelstrike(x0,params); % Find when heelstrike occurs
    kimp = round(timp/dt);

    % Choose which alpha to optimize
    alphaRange = 0:0.01:1;
    
    % Choose teh cost function
    cost_func = 'NRMSE';

    for k = 1:kimp
        for j = 1:length(alphaRange) 
            % The current constant for the convex combination
            alpha = alphaRange(j);
            
            % The initial condition for simulation
            x0 = RdataxRes(:,k);
            
            % Simulate the nonlinear CGB model
            tspan = 0:dt:0.6;
            [~,xnonlin] = ode45(@(t,x) dynamics(t,x,m,a,mH,phi1,phi2), tspan,x0);

            % Find the location of heelstrike
            sw_cond = xnonlin(:,1)+xnonlin(:,2); % Switch condition th1+th2 = 0
            crossIdx = find(sw_cond(2:end).*sw_cond(1:end-1)<0);

            if isempty(crossIdx)
                crossIdx = 1;
            end
            crossIdx = crossIdx(end);
            % The linearization index at alpha = 0.5
            xend = xnonlin(crossIdx,:).';
            
            % Calculate the linearization point xeq
            xeq = x0*(1-alpha) + xend*alpha;
            
            % Linearize around the linearization point
            A = Asymfun(xeq,0);
            Baff = Baffsymfun(xeq,0);
            
            % Simulate the linear system
            xlin = x0;
            for i = 1:kimp-k
                xdot(:,i) = A*xlin(:,i)+Baff;
                xlin(:,i+1) = xlin(:,i)+xdot(:,i)*dt;
            end
            
            % Evaluate how well the linear system fits the data
            J3Data(j,k) = sum(goodnessOfFit([xlin(1,:);xlin(2,:)].',[RdataxRes(1,k:end);RdataxRes(2,k:end)].',cost_func));

            % Evaluate how well the linear system fits the nonlinear system
            J3Model(j,k) = sum(goodnessOfFit([xlin(1,:);xlin(2,:)].',[xnonlin(1:kimp-k+1,1),xnonlin(1:kimp-k+1,2)],cost_func));
        end
    end
end


