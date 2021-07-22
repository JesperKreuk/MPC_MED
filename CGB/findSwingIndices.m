function [LSwingIndices, RSwingIndices] = findSwingIndices(GaitPhaseData)
    leftLegState        = GaitPhaseData.signals.values(:,1);
    rightLegState       = GaitPhaseData.signals.values(:,2);

    % Data valid while the state is 3 and 4, that is the swinging leg
    LswingIdx = find(leftLegState == 3 | leftLegState == 4);
    LswingEndIdx = find(diff(LswingIdx)>1);
    
    RswingIdx = find(rightLegState == 3 | rightLegState == 4);
    RswingEndIdx = find(diff(RswingIdx)>1);

    %% Extract indices of left swings
    LSwingIndices = cell(length(LswingEndIdx),1);
    LSwingIndices{1} = LswingIdx(1):LswingIdx(LswingEndIdx(1));
    for i = 1:length(LswingEndIdx)-1
        LSwingIndices{i+1} = LswingIdx(LswingEndIdx(i)+1):LswingIdx(LswingEndIdx(i+1));
    end
    
    RSwingIndices = cell(length(RswingEndIdx),1);
    RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
    for i = 1:length(RswingEndIdx)-1
        RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
    end
end
