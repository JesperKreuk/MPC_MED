function J = costFunction(uConstant, yref, model, dt_visual)
    uConstant
    
    warning off
    simout = sim(model,'SrcWorkspace','current');
    warning on
    
    stepLength =  simout.RFootPos.signals.values(end,1)-simout.LFootPos.signals.values(end,1)
    J = (stepLength-yref)^2
end
