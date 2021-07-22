function J = costFunctionInit(cGuess,yref, in)
    cGuess
    % Only the first N inputs will be changed
    N = length(cGuess);
    % Convert input to a time series used in the simulation
    t = 0:0.01:1;
    t = t';
    u = [t, cGuess*ones(size(t))];
    
%     options = simset('SrcWorkspace','current');
    warning off
    tic
%     simout = parsim('nms_model','SimulationMode','rapid','SrcWorkspace','current');
    simout = parsim(in, 'ShowProgress', true, 'TransferBaseWorkspaceVariables', 'on') ;    
    toc
    warning on
    
    stepLength = simout.RFootPos.signals.values(end,1)-simout.LFootPos.signals.values(end,1)
    J = (stepLength-yref)^2;
end
