function [modelOpt, ParametersOpt] = greyboxEstimation(dataset, swingNumber, skipSwingPercentage, virGravityDirs)
    if nargin < 3
        skipSwingPercentage = 0;
        virGravityDirs = 2;
    elseif nargin < 4
        virGravityDirs = 2;
    end
    
    %% Load data and parameters
    BodyMechParams
    initialCWParameters
    
    load(dataset,'simout')
    
    % Time
    t = simout.greyEstData.time;
    dt = diff(t(1:2));
    
    % Extract relevant locations
    HATPosxy = simout.greyEstData.signals.values(:,1:2);
    LToePosxy = simout.greyEstData.signals.values(:,3:4);
    LAnklePosxy = simout.greyEstData.signals.values(:,5:6);
    LHeelPosxy = simout.greyEstData.signals.values(:,7:8);
    LKneePosxy = simout.greyEstData.signals.values(:,9:10);
    LHipPosxy = simout.greyEstData.signals.values(:,11:12);
    RToePosxy = simout.greyEstData.signals.values(:,13:14);
    RAnklePosxy = simout.greyEstData.signals.values(:,15:16);
    RHeelPosxy = simout.greyEstData.signals.values(:,17:18);
    RKneePosxy = simout.greyEstData.signals.values(:,19:20);
    RHipPosxy = simout.greyEstData.signals.values(:,21:22);

    % This constant is calculated in calculateSensorData.m
    hipCorrection = 0.0816;
    
    
    
    %% Find heelstrike
    RswingIdx = find(simout.RSwing.signals.values>0);
    RswingEndIdx = find(diff(RswingIdx)>1);

    RSwingIndices = cell(length(RswingEndIdx),1);
    RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
    for i = 1:length(RswingEndIdx)-1
        RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
    end


    allIndices = cell2mat(RSwingIndices(swingNumber));
    Ndata = length(allIndices);
    skipIndices = round(skipSwingPercentage/100*Ndata);
    cutIndices =  allIndices(skipIndices+1:end);
    %% Extract data from swingnumber (x,u)
    [Ldatax, Ldatau, ~] = calculateSensorData(LAnklePosxy,...
                RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,cutIndices);
    data = iddata(Ldatax(:,1:2), Ldatau, dt');

    %% Initial guess
    % Parameters
    mHInit = hatMass;
    mInit = footMass+shankMass+thighMass;
    aInit = (shankMass*shankAnkleToCGDist+thighMass*(shankLength+thighKneeToCG))/mInit;
    phi1Init = 10/180*pi;
    phi2Init = 10/180*pi;
    
    %% Parameter estimation
    Order         = [2 2 4];           % Model orders [ny nu nx]
    
    InitialStates = [Ldatax(1,1); Ldatax(1,2); Ldatax(1,3);Ldatax(1,4)];            % Initial initial states.
    Ts            = 0;                 % Time-continuous system.

    if virGravityDirs == 1
        ParametersInit    = [mInit; aInit; mHInit; phi1Init];         % Initial parameters
        modelInit = idnlgrey('CWdynamics1Dir', Order, ParametersInit, InitialStates, Ts, ...
                    'Name', 'Compass Walker');
    elseif virGravityDirs == 2
        ParametersInit    = [mInit; aInit; mHInit; phi1Init; phi2Init];         % Initial parameters
        modelInit = idnlgrey('CWdynamics2Dir', Order, ParametersInit, InitialStates, Ts, ...
                    'Name', 'Compass Walker');
    else
        disp('Error: Set virGravityDirs equal to 1 or 2')
    end

    % Set bounds on the parameters, allow 50% Deviation from the original
    % value
    modelInit.Parameters(1).Minimum = mInit*0.7;
    modelInit.Parameters(1).Maximum = mInit*1.3;
    modelInit.Parameters(2).Minimum = aInit*0.7;
    modelInit.Parameters(2).Maximum = aInit*1.3;
    modelInit.Parameters(3).Minimum = mHInit*0.7;
    modelInit.Parameters(3).Maximum = mHInit*1.3; 
    modelInit.Parameters(4).Minimum = 3/180*pi;
    modelInit.Parameters(4).Maximum = 25/180*pi;

%     modelInit.Parameters(1).Fixed = true;
    % model.Parameters(2).Fixed = true;
    % model.Parameters(3).Fixed = true;
    if virGravityDirs == 2
        modelInit.Parameters(5).Minimum = 3/180*pi;
        modelInit.Parameters(5).Maximum = 25/180*pi;
    end

    opt = nlgreyestOptions;
    opt.SearchOptions.MaxIterations = 50;
    modelOpt = nlgreyest(data,modelInit,opt);
    mOpt = modelOpt.Parameters(1).Value;
    aOpt = modelOpt.Parameters(2).Value;
    mHOpt = modelOpt.Parameters(3).Value;
    phi1Opt = modelOpt.Parameters(4).Value;
    ParametersOpt = [mOpt,aOpt,mHOpt, phi1Opt];
    if virGravityDirs == 2
        phi2Opt = modelOpt.Parameters(5).Value;     
        ParametersOpt(end+1)= phi2Opt;
    end
    figure
    compare(data, modelOpt);
end