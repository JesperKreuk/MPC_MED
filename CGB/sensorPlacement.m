% in this case study I will try to find a model be evaluating the 10th swing
% of the right leg, so the right leg is the stance leg
clear all
close all
clc

%% Choose sensor
sensor_stance = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively
sensor_hip = 'hip'; % Choose 'hip','hipcor', 'hat', for the hip, hip with correction or the hat respectively
sensor_swing = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively

%% Choose data set
swingNumber = 15;
%% Setup paths and load data and parameters
setup_paths

loadGreyEstData
initialCWParameters
dt_visual = 1e-3;

%% Find heelstrike
GaitPhaseData       = simout.GaitPhaseData;
% [LSwingIndices, RSwingIndices] = findSwingIndices(GaitPhaseData);

RswingIdx = find(simout.RSwing.signals.values>0);
RswingEndIdx = find(diff(RswingIdx)>1);

RSwingIndices = cell(length(RswingEndIdx),1);
RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
for i = 1:length(RswingEndIdx)-1
    RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
end

indices = cell2mat(RSwingIndices(swingNumber));
endIndx = indices(end);

L = 1;
%% Sensor placement
% sensor_stance = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively
% sensor_hip = 'hip'; % Choose 'hip', 'hat', for the hip or the hat respectively
% sensor_swing = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively

figure('DefaultAxesFontSize',11); hold on
for i = 1:3
    % Find hip correction
    if i == 1
        RfootPosxy = RToePosxy;
        LfootPosxy = LToePosxy;
    elseif i == 2
        RfootPosxy = RAnklePosxy;
        LfootPosxy = LAnklePosxy;
    elseif i == 3
        RfootPosxy = RHeelPosxy;
        LfootPosxy = LHeelPosxy;
    end
    % Hip correction = hipPos - hipPosDesired
    % Desired hipPos is in the middle of the two feet upon heelstrike
    hipCorrection = (LHipPosxy(endIndx) + RHipPosxy(endIndx))/2-(RfootPosxy(endIndx)+LfootPosxy(endIndx))/2

    HipPosxy = (LHipPosxy + RHipPosxy)/2 - hipCorrection; 
    
    % Calculate state 
    th1 = asin((HipPosxy(:,1)-LfootPosxy(:,1))/L);
    th2 = asin((HipPosxy(:,1)-RfootPosxy(:,1))/L);

    dth1 = gradient(th1,dt_visual);
    dth2 = gradient(th2,dt_visual);

    % Define x and u
    x = [th1,th2,dth1,dth2];

    % Cut data
    tdata = t(indices);
    Ldatax = x(indices,:);
    Ndata = length(indices);
    swingpercentagedata = linspace(0,100,Ndata);

    subplot(221)
    hold on
    if i == 1
        plot(swingpercentagedata,Ldatax(:,1),'k','LineWidth',1.2)
    elseif i == 2
        plot(swingpercentagedata,Ldatax(:,1),'b--','LineWidth',1.5)
    elseif i == 3
        plot(swingpercentagedata,Ldatax(:,1),'r-.','LineWidth',1.5)
    end
    title('Stance leg')
    ylabel('Angle in rad')
    legend('Toe','Ankle','Heel', 'Location', 'northwest')
    
    subplot(222)
    hold on
    if i == 1
        plot(swingpercentagedata,Ldatax(:,2),'k','LineWidth',1.2)
    elseif i == 2
        plot(swingpercentagedata,Ldatax(:,2),'b--','LineWidth',1.5)
    elseif i == 3
        plot(swingpercentagedata,Ldatax(:,2),'r-.','LineWidth',1.5)
    end
    title('Swing leg')
    
    subplot(223)
    hold on
    if i == 1
        plot(swingpercentagedata,Ldatax(:,3),'k','LineWidth',1.2)
    elseif i == 2
        plot(swingpercentagedata,Ldatax(:,3),'b--','LineWidth',1.5)
    elseif i == 3
        plot(swingpercentagedata,Ldatax(:,3),'r-.','LineWidth',1.5)
    end
    
%     title('Stance leg angle $\theta_1$', 'Interpreter','latex')
    ylabel('Angular velocity in rad/s')
    xlabel('Swing in %')
    
    subplot(224)
    
%     title('$\dot{\theta_2}$', 'Interpreter','latex')
    hold on    
    if i == 1
        plot(swingpercentagedata,Ldatax(:,4),'k','LineWidth',1.2)
    elseif i == 2
        plot(swingpercentagedata,Ldatax(:,4),'b--','LineWidth',1.5)
    elseif i == 3
        plot(swingpercentagedata,Ldatax(:,4),'r-.','LineWidth',1.5)
    end
    ylabel('Angular velocity in rad/s')
    xlabel('Swing in %')

%     v = get(gca,'Position');
%     set(gca,'Position',[v(1) v(2)*1.5 v(3:4)])
    if i == 2
        LFootVisualx = LfootPosxy(indices,1);
        LFootVisualy = zeros(size(LfootPosxy(indices,1)));  %RfootPosxy(indices,2);
        HipVisualx =LFootVisualx + L*sin(Ldatax(:,1));
        HipVisualy = LFootVisualy + L*cos(Ldatax(:,1));
        RFootVisualx = HipVisualx-L*sin(Ldatax(:,2));
        RFootVisualy = HipVisualy-L*cos(Ldatax(:,2));

        LToeVisual = LToePosxy(indices,:);
        LAnkleVisual = LAnklePosxy(indices,:);
        LHeelVisual = LHeelPosxy(indices,:);
        LKneeVisual = LKneePosxy(indices,:);
        LHipVisual = LHipPosxy(indices,:);
        RToeVisual = RToePosxy(indices,:);
        RAnkleVisual = RAnklePosxy(indices,:);
        RHeelVisual = RHeelPosxy(indices,:);
        RKneeVisual = RKneePosxy(indices,:);
        RHipVisual = RHipPosxy(indices,:);

    end
end



figure('DefaultAxesFontSize',11); 
% if sensor_stance == 't' && sensor_swing == 't' && strcmp(sensor_hip,'hip') 
%     title('Measured from toe to hip to toe')
% elseif sensor_stance == 'a' && sensor_swing == 'a' && strcmp(sensor_hip,'hip') 
%     title('Measured from ankle to hip to ankle')
% elseif sensor_stance == 'h' && sensor_swing == 'h' && strcmp(sensor_hip,'hip') 
%     title('Measured from heel to hip to heel')
% elseif sensor_stance == 't' && sensor_swing == 't' && strcmp(sensor_hip,'hat') 
%     title('Measured from toe to hat to toe')
% elseif sensor_stance == 'a' && sensor_swing == 'a' && strcmp(sensor_hip,'hat') 
%     title('Measured from ankle to hat to ankle')
% elseif sensor_stance == 'h' && sensor_swing == 'h' && strcmp(sensor_hip,'hat') 
%     title('Measured from heel to hat to heel')
% elseif sensor_stance == 't' && sensor_swing == 'a' && strcmp(sensor_hip,'hip') 
%     title('Measured from toe to hip to ankle')
% end
hold on

% set(gca,'XLim',[-0.7 1.7], 'YLim',[-0.1 1])
axis equal
% grid on

set(gca,'XTick',[],'YTick',[])
% xlabel('{\it p_x} in meters')
% ylabel('{\it p_y} in meters')
% xlabel('$p_x$ in meters', 'Interpreter','latex','FontSize',14)
% ylabel('$p_y$ in meters', 'Interpreter','latex','FontSize',14)
i = 1;

% CoM1 = plot(LCoMPosxy(indices(i),1),LCoMPosxy(indices(i),2),'r*');
% CoM2 = plot(RCoMPosxy(indices(i),1),RCoMPosxy(indices(i),2),'r*');
% CoM3 = plot(HATCoMPosxy(indices(i),1),HATCoMPosxy(indices(i),2),'r*');
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
% 
% % Calculate the hip correction, which is the displacement in x position of
% % the hip to make the compass-gait biped hit the ground at the same time as
% % the model.
% RFootEnd = RfootPosxy(indices(end),1);
% LFootEnd = LfootPosxy(indices(end),1);
% RHipPosEnd = RHipPosxy(indices(end),1);
% LHipPosEnd = LHipPosxy(indices(end),1);
% hipPosEnd = (LHipPosEnd+RHipPosEnd)/2;
% hipCorrection = hipPosEnd-(LFootEnd+RFootEnd)/2;
% 
% if strcmp(sensor_hip,'hip') 
%     HipPosxy = (LHipPosxy + RHipPosxy)/2;
% elseif strcmp(sensor_hip,'hipcor')      
%     HipPosxy = (LHipPosxy + RHipPosxy)/2-hipCorrection;
% elseif strcmp(sensor_hip,'hat') 
%     HipPosxy = HATPosxy;
% elseif strcmp(sensor_hip,'chat') 
%     HipPosxy = HATCoMPosxy;
% end
% 
% if strcmp(sensor_hip,'chat') && strcmp(sensor_stance,'c') && strcmp(sensor_swing,'c') 
%     HipPosxy = (LHipPosxy + RHipPosxy)/2;
%     th1 = atan((HipPosxy(:,1)-LCoMPosxy(:,1))./(HipPosxy(:,2)-LCoMPosxy(:,2)));
%     th2 = atan((HipPosxy(:,1)-RCoMPosxy(:,1))./(HipPosxy(:,2)-RCoMPosxy(:,2)));
% else
%     th1 = asin((HipPosxy(:,1)-LfootPosxy(:,1))/L);
%     th2 = asin((HipPosxy(:,1)-RfootPosxy(:,1))/L);
% end
% 
% % Calculate dth1 and dth2
% dth1 = gradient(th1,dt_visual);
% dth2 = gradient(th2,dt_visual);
% 
% % Define x and u
% x = [th1,th2,dth1,dth2];
% u = cos(x(:,1:2));
% 
% % Cut data
% tdata = t(indices);
% Ldatax = x(indices,:);
% Ldatau = u(indices,:);
% Ndata = length(indices);
% swingpercentagedata = linspace(0,100,Ndata);
% 
% data = iddata(Ldatax, Ldatau, dt_visual');
% 
% 
% %% Validate data
% set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
% set(groot, 'defaultLegendInterpreter','latex');
% 
% figure
% subplot(211); hold on
% if sensor_stance == 't' && sensor_swing == 't' && strcmp(sensor_hip,'hip') 
%     title('Measured from toe to hip to toe')
% elseif sensor_stance == 'a' && sensor_swing == 'a' && strcmp(sensor_hip,'hip') 
%     title('Measured from ankle to hip to ankle')
% elseif sensor_stance == 'h' && sensor_swing == 'h' && strcmp(sensor_hip,'hip') 
%     title('Measured from heel to hip to heel')
% elseif sensor_stance == 't' && sensor_swing == 't' && strcmp(sensor_hip,'hat') 
%     title('Measured from toe to hat to toe')
% elseif sensor_stance == 'a' && sensor_swing == 'a' && strcmp(sensor_hip,'hat') 
%     title('Measured from ankle to hat to ankle')
% elseif sensor_stance == 'h' && sensor_swing == 'h' && strcmp(sensor_hip,'hat') 
%     title('Measured from heel to hat to heel')
% elseif sensor_stance == 't' && sensor_swing == 'a' && strcmp(sensor_hip,'hip') 
%     title('Measured from toe to hip to ankle')
% end
% plot(swingpercentagedata,Ldatax(:,1:2))
% ylabel('Angle in rad')
% legend('$\theta_1$', '$\theta_2$','location','east')
% 
% subplot(212)
% plot(swingpercentagedata,Ldatax(:,3:4))
% ylabel('Angular velocity in rad/s')
% xlabel('Swing in %')
% legend('$\dot{\theta_1}$', '$\dot{\theta_2}$','location', 'southeast')
% 
% v = get(gca,'Position');
% set(gca,'Position',[v(1) v(2)*1.5 v(3:4)])
% % saveas(gcf,strcat(upper(sensor_stance),sensor_hip,upper(sensor_swing)),'png')
% % saveas(gcf,strcat(upper(sensor_stance),sensor_hip,upper(sensor_swing)),'epsc')
% 
%% Visualize walker
% % For left swing
% RFootVisualx = RfootPosxy(indices,1);
% RFootVisualy = zeros(size(RfootPosxy(indices,2)));  %RfootPosxy(indices,2);
% HipVisualx =RFootVisualx + LInit*sin(Ldatax(:,2));
% HipVisualy = RFootVisualy + LInit*cos(Ldatax(:,2));
% LFootVisualx = HipVisualx-LInit*sin(Ldatax(:,1));
% LFootVisualy = HipVisualy-LInit*cos(Ldatax(:,1));

% For right swing
LFootVisualx = LfootPosxy(indices,1);
LFootVisualy = zeros(size(LfootPosxy(indices,1)));  %RfootPosxy(indices,2);
HipVisualx =LFootVisualx + L*sin(Ldatax(:,1));
HipVisualy = LFootVisualy + L*cos(Ldatax(:,1));
RFootVisualx = HipVisualx-L*sin(Ldatax(:,2));
RFootVisualy = HipVisualy-L*cos(Ldatax(:,2));

LToeVisual = LToePosxy(indices,:);
LAnkleVisual = LAnklePosxy(indices,:);
LHeelVisual = LHeelPosxy(indices,:);
LKneeVisual = LKneePosxy(indices,:);
LHipVisual = LHipPosxy(indices,:);
RToeVisual = RToePosxy(indices,:);
RAnkleVisual = RAnklePosxy(indices,:);
RHeelVisual = RHeelPosxy(indices,:);
RKneeVisual = RKneePosxy(indices,:);
RHipVisual = RHipPosxy(indices,:);


figure('DefaultAxesFontSize',11); 
% if sensor_stance == 't' && sensor_swing == 't' && strcmp(sensor_hip,'hip') 
%     title('Measured from toe to hip to toe')
% elseif sensor_stance == 'a' && sensor_swing == 'a' && strcmp(sensor_hip,'hip') 
%     title('Measured from ankle to hip to ankle')
% elseif sensor_stance == 'h' && sensor_swing == 'h' && strcmp(sensor_hip,'hip') 
%     title('Measured from heel to hip to heel')
% elseif sensor_stance == 't' && sensor_swing == 't' && strcmp(sensor_hip,'hat') 
%     title('Measured from toe to hat to toe')
% elseif sensor_stance == 'a' && sensor_swing == 'a' && strcmp(sensor_hip,'hat') 
%     title('Measured from ankle to hat to ankle')
% elseif sensor_stance == 'h' && sensor_swing == 'h' && strcmp(sensor_hip,'hat') 
%     title('Measured from heel to hat to heel')
% elseif sensor_stance == 't' && sensor_swing == 'a' && strcmp(sensor_hip,'hip') 
%     title('Measured from toe to hip to ankle')
% end
hold on

% set(gca,'XLim',[-0.7 1.7], 'YLim',[-0.1 1])
axis equal
% grid on

set(gca,'XTick',[],'YTick',[])
% xlabel('{\it p_x} in meters')
% ylabel('{\it p_y} in meters')
% xlabel('$p_x$ in meters', 'Interpreter','latex','FontSize',14)
% ylabel('$p_y$ in meters', 'Interpreter','latex','FontSize',14)
i = 1;

% CoM1 = plot(LCoMPosxy(indices(i),1),LCoMPosxy(indices(i),2),'r*');
% CoM2 = plot(RCoMPosxy(indices(i),1),RCoMPosxy(indices(i),2),'r*');
% CoM3 = plot(HATCoMPosxy(indices(i),1),HATCoMPosxy(indices(i),2),'r*');
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
drawnow
% 
% % myVideo = VideoWriter(strcat(upper(sensor_stance),sensor_hip,upper(sensor_swing),'Video')); %open video file
% % myVideo.FrameRate = 100;  %can adjust this, 5 - 10 works well for me
% % open(myVideo)
% 
% for i = 1:length(HipVisualx)
%     delete(sixLinkWalker)
%     delete(walker)
%     delete(heel1)
%     delete(heel2)
% %     delete(CoM1)
% %     delete(CoM2)
% %     delete(CoM3)
% %     CoM1 = plot(LCoMPosxy(indices(i),1),LCoMPosxy(indices(i),2),'r*');
% %     CoM2 = plot(RCoMPosxy(indices(i),1),RCoMPosxy(indices(i),2),'r*');
% %     CoM3 = plot(HATCoMPosxy(indices(i),1),HATCoMPosxy(indices(i),2),'r*');
%     walker = plot([LFootVisualx(i),HipVisualx(i),RFootVisualx(i)],[LFootVisualy(i),HipVisualy(i),RFootVisualy(i)],'o-','Color','r');
%     sixLinkWalker = plot([LToeVisual(i,1), LAnkleVisual(i,1),LKneeVisual(i,1),LHipVisual(i,1), ...
%                        RHipVisual(i,1),RKneeVisual(i,1),RAnkleVisual(i,1),RToeVisual(i,1)], ...
%                        [LToeVisual(i,2),LAnkleVisual(i,2),LKneeVisual(i,2),LHipVisual(i,2), ...
%                        RHipVisual(i,2),RKneeVisual(i,2),RAnkleVisual(i,2),RToeVisual(i,2)],'o-','Color','k');
%     heel1 = plot([LHeelVisual(i,1), LAnkleVisual(i,1)], ...
%             [LHeelVisual(i,2),LAnkleVisual(i,2)],'o-','Color','k');
%     heel2 = plot([RHeelVisual(i,1), RAnkleVisual(i,1)], ...
%             [RHeelVisual(i,2),RAnkleVisual(i,2)],'o-','Color','k');
%     drawnow
% %     frame = getframe(gcf); %get frame
% %     writeVideo(myVideo, frame);
% end
% % close(myVideo)
% 
% % close(myVideo)
