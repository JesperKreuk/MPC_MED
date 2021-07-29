function hipCorrection = calculateHipCorrection(dataset,swingNumber,position)
    load(dataset, 'simout')
    loadGreyboxPositions
    
    if nargin < 3
        % Use the ankles
        RfootPosxy = RAnklePosxy;
        LfootPosxy = LAnklePosxy;
    else
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
            disp('Warning, incorrect position argument')
        end
    end

    indices = findSwingIndices(dataset,swingNumber);
    endIndx = indices(end);

    
    % Hip correction = hipPos - hipPosDesired
    % Desired hipPos is in the middle of the two feet upon heelstrike
    hipCorrection = (LHipPosxy(endIndx) + RHipPosxy(endIndx))/2-(RfootPosxy(endIndx)+LfootPosxy(endIndx))/2;
end