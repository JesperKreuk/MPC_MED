%{
This function validates the model of the CGB walker on another swing from a
dataset. The same dataset is used as for the greybox estimation.

Arguments:
* dataset: the dataset containing the positions required for the greybox
    estimation
* swingNumber: which swing of the data is used for greybox estimation
* modelOpt: The optimized CGB model (with an initial state for simulation)

There is no output besides the a figure that compares the model with the
data 

Author: Jesper Kreuk
%}

function greyboxValidation(dataset, swingNumber, modelOpt)
    % Load data
    load(dataset, 'simout');
    loadGreyboxPositions;
    
    % Calculate states for the correct swing and save as iddata variable
    hipCorrection = calculateHipCorrection(dataset,10); % Swing number 10 is used because this is the identification swing
    swingIndices = findSwingIndices(dataset,swingNumber);
    [RdataxVal, RdatauVal] = calculateSensorData(LAnklePosxy,...
                    RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,swingIndices);
    dt_id = 1e-3; % step time of the identification data
    dataVal = iddata(RdataxVal(:,1:2), RdatauVal, dt_id);

    % Set the correct initial conditions for simulation of the model
    modelOpt.Initialstates(1).Value = RdataxVal(1,1);
    modelOpt.Initialstates(2).Value = RdataxVal(1,2);
    modelOpt.Initialstates(3).Value = RdataxVal(1,3);
    modelOpt.Initialstates(4).Value = RdataxVal(1,4);
    
    % Plot the figure to compare the validation data with the model
    figure
    compare(dataVal, modelOpt);
end