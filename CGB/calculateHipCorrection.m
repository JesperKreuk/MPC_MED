%{
This function writes the linear MLD system of the CGB model given the 
linear continuous time matrices such the MLD system has the following form
           x(k+1) = A*x(k) + Bu*u + Baux*w + Baff
           y(k) = C*x(k) 
For more details, see Appendix D of the corresponding thesis

Arguments:
* dataset: the dataset containing the positions required for the greybox
    estimation
* swingNumber: which swing of the data is used for greybox estimation
* position: which position on the foot is tracked by the sensors

Output: 
* hipCorrection: a constant offset in the px direction of the hip of the
    CGB model with respect to the hip of the NMS model

Author: Jesper Kreuk
%}

function hipCorrection = calculateHipCorrection(dataset,swingNumber,position)
    % Load data and positions
    load(dataset, 'simout')
    loadGreyboxPositions
    
    if nargin < 3
        % Use the ankles if no position is specified
        RfootPosxy = RAnklePosxy;
        LfootPosxy = LAnklePosxy;
    else
        % Find the positions of the specified bodypart
        if strcmp(position,'toe')
            RfootPosxy = RToePosxy;
            LfootPosxy = LToePosxy;
        elseif strcmp(position,'heel')
            RfootPosxy = RHeelPosxy;
            LfootPosxy = LHeelPosxy;
        elseif strcmp(position,'ankle')
            RfootPosxy = RAnklePosxy;
            LfootPosxy = LAnklePosxy;
        else
            % Give a warning of an unknown position is given
            disp('Warning, incorrect position argument')
        end
    end

    % Find the index where heelstrike occurs such that the hipCorrection
    % can be calculated
    indices = findSwingIndices(dataset,swingNumber);
    endIndx = indices(end);

    % Hip correction = hipPos - hipPosDesired
    % Desired hipPos is in the middle of the two feet upon heelstrike
    hipCorrection = (LHipPosxy(endIndx) + RHipPosxy(endIndx))/2-(RfootPosxy(endIndx)+LfootPosxy(endIndx))/2;
end