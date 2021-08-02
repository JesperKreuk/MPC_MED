%{
This function is an extension of calculateSensorData function. It is
capable of calculating the states as well, but multiple positions are
possible, like the toe, ankle and hip as well as a visualisation of how the
positions fit the NMS model.

Arguments:
* dataset: the dataset containing the positions required for the greybox
    estimation
* swingNumber: which swing of the data is used for greybox estimation
* position: which position on the foot is tracked by the sensors

Output: 
* Rdatax: Matrix containing the states [th1;th2;dth1;dth2] where th is
    short for theta
* swingPercentageData: the normalised time vector of the chosen swing

Author: Jesper Kreuk
%}

function [Rdatax, swingPercentageData] = positionTracking(dataset,swingNumber, position, visualise)
    if nargin < 4
        visualise = 0;
    end
        
    load(dataset, 'simout');
    loadGreyboxPositions;
    
    dt_id = 1e-3;

    swingIndices = findSwingIndices(dataset,swingNumber);
    hipCorrection = calculateHipCorrection(dataset,swingNumber,position);
    
    L = 1;
    %% Position to track
    if strcmp(position,'toe')
        RfootPosxy = RToePosxy;
        LfootPosxy = LToePosxy;
    elseif  strcmp(position,'ankle')
        RfootPosxy = RAnklePosxy;
        LfootPosxy = LAnklePosxy;
    elseif  strcmp(position,'heel')
        RfootPosxy = RHeelPosxy;
        LfootPosxy = LHeelPosxy;
    end

    [Rdatax, ~, Ndata] = calculateSensorData(LfootPosxy,...
        RfootPosxy,LHipPosxy,RHipPosxy,hipCorrection,swingIndices);

    swingPercentageData = linspace(0,100,Ndata);

    %% Walker example
    if visualise
        LFootVisualx = LfootPosxy(swingIndices,1);
        LFootVisualy = zeros(size(LfootPosxy(swingIndices,1)));  %RfootPosxy(swingIndices,2);
        HipVisualx =LFootVisualx + L*sin(Rdatax(:,1));
        HipVisualy = LFootVisualy + L*cos(Rdatax(:,1));
        RFootVisualx = HipVisualx-L*sin(Rdatax(:,2));
        RFootVisualy = HipVisualy-L*cos(Rdatax(:,2));

        LToeVisual = LToePosxy(swingIndices,:);
        LAnkleVisual = LAnklePosxy(swingIndices,:);
        LHeelVisual = LHeelPosxy(swingIndices,:);
        LKneeVisual = LKneePosxy(swingIndices,:);
        LHipVisual = LHipPosxy(swingIndices,:);
        RToeVisual = RToePosxy(swingIndices,:);
        RAnkleVisual = RAnklePosxy(swingIndices,:);
        RHeelVisual = RHeelPosxy(swingIndices,:);
        RKneeVisual = RKneePosxy(swingIndices,:);
        RHipVisual = RHipPosxy(swingIndices,:);

        figure('DefaultAxesFontSize',11); 
        hold on
        axis equal
        
        i = 1;
        set(gca,'XTick',[],'YTick',[])
        walker = plot([LFootVisualx(i),HipVisualx(i),RFootVisualx(i)],[LFootVisualy(i),HipVisualy(i),RFootVisualy(i)],'o-','Color','r');
        sixLinkWalker = plot([LToeVisual(i,1), LAnkleVisual(i,1),LKneeVisual(i,1),LHipVisual(i,1), ...
                               RHipVisual(i,1),RKneeVisual(i,1),RAnkleVisual(i,1),RToeVisual(i,1)], ...
                               [LToeVisual(i,2),LAnkleVisual(i,2),LKneeVisual(i,2),LHipVisual(i,2), ...
                               RHipVisual(i,2),RKneeVisual(i,2),RAnkleVisual(i,2),RToeVisual(i,2)],'o-','Color','k');
        heel1 = plot([LHeelVisual(i,1), LAnkleVisual(i,1)], ...
                [LHeelVisual(i,2),LAnkleVisual(i,2)],'o-','Color','k');
        heel2 = plot([RHeelVisual(i,1), RAnkleVisual(i,1)], ...
                [RHeelVisual(i,2),RAnkleVisual(i,2)],'o-','Color','k');   

        legend([sixLinkWalker, walker],'NMS model','CGB model')
    end
end
