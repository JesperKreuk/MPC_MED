%{
This function finds the indices corresponding the the swing of a right leg
Author: Jesper Kreuk
%}

function indices = findSwingIndices(dataset,swingNumber)
    load(dataset, 'simout')
    RswingIdx = find(simout.RSwing.signals.values>0);
    RswingEndIdx = find(diff(RswingIdx)>1);

    RSwingIndices = cell(length(RswingEndIdx),1);
    RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
    for i = 1:length(RswingEndIdx)-1
        RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
    end
    indices = cell2mat(RSwingIndices(swingNumber));
end
