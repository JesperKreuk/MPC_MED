function [stepLength, indices] = findStepLength(simout, swingNumber)
    % Evaluate the step length 
    LAnklePosxy = simout.jointData.signals.values(:,3:4);
    RAnklePosxy = simout.jointData.signals.values(:,7:8);

    RswingIdx = find(simout.RSwing.signals.values>0);
    RswingEndIdx = find(diff(RswingIdx)>1);

    RSwingIndices = cell(length(RswingEndIdx),1);
    RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
    for i = 1:length(RswingEndIdx)-1
        RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
    end

    indices = cell2mat(RSwingIndices(swingNumber));

    finalIndx = indices(end);
    stepLength =  RAnklePosxy(finalIndx) - LAnklePosxy(finalIndx)
end