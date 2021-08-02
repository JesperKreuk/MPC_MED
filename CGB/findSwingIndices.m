%{
This function finds the indices corresponding the the swing of a right leg

Arguments:
* dataset: the dataset containing the positions required for the greybox
    estimation
* swingNumber: which swing of the data is used 

Output: 
* swingIndices: the indices that describe the evalauted swing 

Author: Jesper Kreuk
%}

function swingIndices = findSwingIndices(dataset,swingNumber)
    % Load the dataset
    load(dataset, 'simout')
    
    % Find which values of RSwing are 1, these are swing indices
    RswingIdx = find(simout.RSwing.signals.values>0);
    
    % Where the difference is l, this is where toe off and heel strike
    % occur
    RswingEndIdx = find(diff(RswingIdx)>1);

    % Make cell array where each cell contains all indices of a single
    % swing
    RSwingIndices = cell(length(RswingEndIdx),1);
    RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
    for i = 1:length(RswingEndIdx)-1
        RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
    end
    
    % Return only the indices of the given swingNumber
    swingIndices = cell2mat(RSwingIndices(swingNumber));
end
