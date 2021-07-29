%{
This function does a nonlinear greybox estimation of the CGB model. For this
it uses the data of a certain swing number. This data is from the 3D NMS
model of song.

Arguments:
* dataset: the dataset containing the positions required for the greybox
    estimation
* swingNumber: which swing of the data is used for greybox estimation

Output: 
* ParametersOpt: The optimized parameters [m, a, mH, phi1, phi2] of the CGB
    model

Author: Jesper Kreuk
%}

function [parametersOpt, modelOpt] = greyboxEstimation(dataset, swingNumber)
    %% Load data and parameters
    BodyMechParams % This contains the body masses
        
    load(dataset,'simout');
    loadGreyboxPositions

    swingIndices = findSwingIndices(dataset,swingNumber);
    hipCorrection = calculateHipCorrection(dataset,swingNumber);
    %% Extract data from swingnumber (x,u)
    [Rdatax, Rdatau, ~] = calculateSensorData(LAnklePosxy,...
                RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,swingIndices);
    dt_id = 1e-3; % step time of the identification data
    data = iddata(Rdatax(:,1:2), Rdatau, dt_id);

    %% Initial guess
    % Estimate the masses and distances. The virtual gravity angles are
    % chosen arbitrary
    mHInit = hatMass;
    mInit = footMass+shankMass+thighMass;
    aInit = (shankMass*shankAnkleToCGDist+thighMass*(shankLength+thighKneeToCG))/mInit;
    phi1Init = 10/180*pi;
    phi2Init = 10/180*pi;
    
    %% Parameter estimation
    Order         = [2 2 4];           % Model orders [ny nu nx]
    
    InitialStates = [Rdatax(1,1); Rdatax(1,2); Rdatax(1,3);Rdatax(1,4)];            % Initial initial states.
    Ts            = 0;                 % Time-continuous system.


    ParametersInit    = [mInit; aInit; mHInit; phi1Init; phi2Init];         % Initial parameters
    modelInit = idnlgrey('dynamicsGreybox', Order, ParametersInit, InitialStates, Ts, ...
                'Name', 'Compass Walker');


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
    modelInit.Parameters(5).Minimum = 3/180*pi;
    modelInit.Parameters(5).Maximum = 25/180*pi;
    

    opt = nlgreyestOptions;
    opt.SearchOptions.MaxIterations = 50;
    modelOpt = nlgreyest(data,modelInit,opt);
    mOpt = modelOpt.Parameters(1).Value;
    aOpt = modelOpt.Parameters(2).Value;
    mHOpt = modelOpt.Parameters(3).Value;
    phi1Opt = modelOpt.Parameters(4).Value;
    phi2Opt = modelOpt.Parameters(5).Value;  
    parametersOpt = [mOpt,aOpt,mHOpt, phi1Opt, phi2Opt];   
    figure
    compare(data, modelOpt);
end