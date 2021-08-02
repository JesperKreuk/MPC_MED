%{
This function calculates the step length of a swing given the output of the 
simulation and the number of the swing. The step length is approximated by
the ankle to ankle distance at the end of the swing.

Arguments:
* simout: a matrix containing a sequence of states obtained from
    simulating the nonlinear CGB model
* swingNumber: which swing of the data is used 

Output: 
* stepLength: the ankle to ankle distance at heel strike in m
* indices: the indices that describe the evalauted swing 


Author: Jesper Kreuk
%}
function [stepLength, swingIndices] = findStepLength(simout, swingNumber)
    % Extract the ankle positions
    LAnklePosxy = simout.jointData.signals.values(:,3:4);
    RAnklePosxy = simout.jointData.signals.values(:,7:8);

    % Find the index of heelstrike 
    RswingIdx = find(simout.RSwing.signals.values>0);
    RswingEndIdx = find(diff(RswingIdx)>1);

    RSwingIndices = cell(length(RswingEndIdx),1);
    RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
    for i = 1:length(RswingEndIdx)-1
        RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
    end
    swingIndices = cell2mat(RSwingIndices(swingNumber));
    finalIndx = swingIndices(end);
    
    % Calculate the ankle to ankle distance
    stepLength =  RAnklePosxy(finalIndx) - LAnklePosxy(finalIndx)
end