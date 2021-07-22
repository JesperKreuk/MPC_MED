function [uopt, Jopt, uPoints, JPoints,flag] = findOptimalControl(umax, yref, model, disturbances, distStart, distStop, controlStart)
    nms_model_MechInit;
    nms_model_ControlInit;
    initState;

    dt_visual = -1;
    %% Find optimization region
    % First check u = 0, umax and umax/2 Nm
    uPoints = [umax/4;3*umax/4;umax/2];
    JPoints = zeros(3,1);
    for i = 1:3
        uConstant = uPoints(i);
        warning off
        simout = sim(model,'SrcWorkspace','current');
        warning on
        stepLength =  simout.RFootPos.signals.values(end,1)-simout.LFootPos.signals.values(end,1);
        JPoints(i) = (stepLength-yref)^2;
    end
    p = polyfit(uPoints(end-2:end),JPoints(end-2:end),2);
    uPoints(end+1) = -p(2)/(2*p(1));
    while 1
        uConstant = uPoints(end);
        warning off
        simout = sim(model,'SrcWorkspace','current');
        warning on
        stepLength =  simout.RFootPos.signals.values(end,1)-simout.LFootPos.signals.values(end,1);
        JPoints(end+1) = (stepLength-yref)^2;
        if JPoints(end) < 1e-8
            if uPoints(end) < 0
               uopt = 0;
               Jopt = JPoints(end);
               flag = 0;
            elseif uPoints(end) > umax
               uopt = 5;
               Jopt = JPoints(end);
               flag = 0;
            else
                uopt = uPoints(end);
                Jopt = JPoints(end);
                flag = 0;
            end
            disp('Found optimal input')
            break
        elseif length(uPoints) > 15
            [~,indx] = min(JPoints);
            uopt = uPoints(indx);
            Jopt = JPoints(indx);
            flag = 1;
            NumberOfIterations = length(uPoints)
            break
        end
        p = polyfit(uPoints(end-2:end),JPoints(end-2:end),2);
        uPoints(end+1) = -p(2)/(2*p(1));
    end
end
