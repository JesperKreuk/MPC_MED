% clear all
% close all
% clc

%% Load data
load('nonlincontrol6uop3.mat')

% Time
simout = simout_nonlin;
t = simout.jointData.time;

% Extract relevant locations
HATPosxy = simout.jointData.signals.values(:,1:2);
LToePosxy = simout.jointData.signals.values(:,3:4);
LAnklePosxy = simout.jointData.signals.values(:,5:6);
LHeelPosxy = simout.jointData.signals.values(:,7:8);
LKneePosxy = simout.jointData.signals.values(:,9:10);
LHipPosxy = simout.jointData.signals.values(:,11:12);
RToePosxy = simout.jointData.signals.values(:,13:14);
RAnklePosxy = simout.jointData.signals.values(:,15:16);
RHeelPosxy = simout.jointData.signals.values(:,17:18);
RKneePosxy = simout.jointData.signals.values(:,19:20);
RHipPosxy = simout.jointData.signals.values(:,21:22);

%% Set parameter
dt = 1e-2;
m = ParametersOpt(1);
a =  ParametersOpt(2);
mH =  ParametersOpt(3);
phi1 =  ParametersOpt(4);
if length(ParametersOpt)>4
    phi2 =  ParametersOpt(5);
else
    phi2 = phi1;
end
L = 1;
b = L-a;
g = 9.81;
params.a = a;
params.L = L;
params.b = b;
params.m = m;
params.mH = mH;
params.g = g;
params.phi1 = phi1;
params.phi2 = phi2;
params.dt = dt;

% This constant is calculated in calculateSensorData.m
hipCorrection = 0.0816;

%% Find heelstrike
swingNumber = 6;
GaitPhaseData       = simout.GaitPhaseDataControl;
[LSwingIndices, RSwingIndices] = findSwingIndices(GaitPhaseData);

indices = cell2mat(RSwingIndices(swingNumber));

%% Compass-gait biped
HipPosxy = (LHipPosxy + RHipPosxy)/2-hipCorrection;

% Theta 1 is always the left healthy and theta 2 is always the right prosthetic leg
th1 = asin((HipPosxy(:,1)-LAnklePosxy(:,1))/L);
th2 = asin((HipPosxy(:,1)-RAnklePosxy(:,1))/L);

% Calculate dth1 and dth2
dth1 = gradient(th1,dt);
dth2 = gradient(th2,dt);

% Define x and u
x = [th1,th2,dth1,dth2];
%% Simulation points 
Rdatax = x(indices,:);
% stanceLeg 

LFootVisualx = LAnklePosxy(indices,1);
LFootVisualy = zeros(size(LAnklePosxy(indices,2)));  %RfootPosxy(indices,2);
HipVisualx =LFootVisualx + L*sin(Rdatax(:,1));
HipVisualy = LFootVisualy + L*cos(Rdatax(:,1));
RFootVisualx = HipVisualx-L*sin(Rdatax(:,2));
RFootVisualy = HipVisualy-L*cos(Rdatax(:,2));

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

%% Control plot
% 
controlInput = simout.controlData.signals.values;
figure
plot(controlInput(indices))
xlabel('time step k')
ylabel('input u')
title('torque on leg') 
%% Simulation plot
yref = 0.65;
ref = LFootVisualx+yref;

figure
hold on
i=1;
refPoint = plot(ref(i),0,'b*');
walker = plot([LFootVisualx(i),HipVisualx(i),RFootVisualx(i)],[LFootVisualy(i),HipVisualy(i),RFootVisualy(i)],'o-','Color','r');

sixLinkWalker = plot([LToeVisual(i,1), LAnkleVisual(i,1),LKneeVisual(i,1),LHipVisual(i,1), ...
                       RHipVisual(i,1),RKneeVisual(i,1),RAnkleVisual(i,1),RToeVisual(i,1)], ...
                       [LToeVisual(i,2),LAnkleVisual(i,2),LKneeVisual(i,2),LHipVisual(i,2), ...
                       RHipVisual(i,2),RKneeVisual(i,2),RAnkleVisual(i,2),RToeVisual(i,2)],'o-','Color','k');
heel1 = plot([LHeelVisual(i,1), LAnkleVisual(i,1)], ...
        [LHeelVisual(i,2),LAnkleVisual(i,2)],'o-','Color','k');
heel2 = plot([LHeelVisual(i,1), LAnkleVisual(i,1)], ...
        [LHeelVisual(i,2),LAnkleVisual(i,2)],'o-','Color','k');            
drawnow
axis equal

for i = 1:length(LToeVisual)
    delete(refPoint)
    delete(walker)
    delete(sixLinkWalker)
    delete(heel1)
    delete(heel2)
    refPoint = plot(ref(i),0,'b*');
    walker = plot([LFootVisualx(i),HipVisualx(i),RFootVisualx(i)],[LFootVisualy(i),HipVisualy(i),RFootVisualy(i)],'o-','Color','r');
    
    sixLinkWalker = plot([LToeVisual(i,1), LAnkleVisual(i,1),LKneeVisual(i,1),LHipVisual(i,1), ...
                       RHipVisual(i,1),RKneeVisual(i,1),RAnkleVisual(i,1),RToeVisual(i,1)], ...
                       [LToeVisual(i,2),LAnkleVisual(i,2),LKneeVisual(i,2),LHipVisual(i,2), ...
                       RHipVisual(i,2),RKneeVisual(i,2),RAnkleVisual(i,2),RToeVisual(i,2)],'o-','Color','k');
    heel1 = plot([LHeelVisual(i,1), LAnkleVisual(i,1)], ...
            [LHeelVisual(i,2),LAnkleVisual(i,2)],'o-','Color','k');
    heel2 = plot([RHeelVisual(i,1), RAnkleVisual(i,1)], ...
            [RHeelVisual(i,2),RAnkleVisual(i,2)],'o-','Color','k');
    drawnow
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);
    pause(0.1)
end

%% Step length no control
finalIndx = indices(end);
th1end = x(finalIndx,1);
th2end = x(finalIndx,2);
strideLength =  L*sin(th1end)-L*sin(th2end)
%% Find step lengths
% strideLengths = zeros(6,1);
% for i = 6:12 %Evaluate swing 6 to 12
%     indices = cell2mat(RSwingIndices(i));
%     th1swing = th1(indices);
%     th2swing = th2(indices);
%     finalIndx = indices(end);
% %     heel = RHeelPosxy(finalIndx,1);
% %     toe = LToePosxy(finalIndx,1);
%     th1end = x(finalIndx,1);
%     th2end = x(finalIndx,2);
%     strideLengths(i-5) =  L*sin(th1end)-sin(th2end); 
% %     strideLengths(i-5,2) =  heel-toe; 
% end
% strideLengths
% 
% %% Evaluate state
% finalSwingIndx = 537:592;
% xcontrol = simout.stateData.signals.values(finalSwingIndx,:);
% ucontrol = simout.controlData.signals.values(finalSwingIndx,:);
% 
% 
% i = 3;
% x0 = xcontrol(i,1:4).';
% % Check if they are the same
% uopt = -NMPC_mex(x0,params)
% uUsed = ucontrol(i)

% %% Get to the bottom of the weird spike
% xcontrol(1:3,1:4)
% x0
% timp = heelstrike(x0, params);
% kimp = round(timp/dt);
% [y,uopt] = NMPC(x0,params)
% % x-xcontrol
% 
% %% Simulation
% xk = x0;
% for i = 1:kimp
%     k1 = dt*CWodefunInput(xk,uopt(i),m,a,mH,phi1,phi2);
%     k2 = dt*CWodefunInput(xk+k1/2,uopt(i),m,a,mH,phi1,phi2);
%     k3 = dt*CWodefunInput(xk+k2/2,uopt(i),m,a,mH,phi1,phi2);
%     k4 = dt*CWodefunInput(xk+k3,uopt(i),m,a,mH,phi1,phi2);
%     xopt(:,i) = xk + (k1+2*k2+2*k3+k4)/6;
%     xk = xopt(:,i);
% end
% 
% xk = x0;
% for i = 1:kimp
%     k1 = dt*CWodefunInput(xk,0,m,a,mH,phi1,phi2);
%     k2 = dt*CWodefunInput(xk+k1/2,0,m,a,mH,phi1,phi2);
%     k3 = dt*CWodefunInput(xk+k2/2,0,m,a,mH,phi1,phi2);
%     k4 = dt*CWodefunInput(xk+k3,0,m,a,mH,phi1,phi2);
%     xidle(:,i) = xk + (k1+2*k2+2*k3+k4)/6;
%     xk = xidle(:,i);
% end
% 
% L = 1;
% Nsim = length(xopt);
% 
% figure;
% hold on
% LFootVisualx = zeros(Nsim,1);
% LFootVisualy = zeros(Nsim,1);  %RfootPosxy(indices,2);
% HipVisualx =LFootVisualx + L*sin(xopt(1,:).');
% HipVisualy = LFootVisualy + L*cos(xopt(1,:).');
% RFootVisualx = HipVisualx-L*sin(xopt(2,:).');
% RFootVisualy = HipVisualy-L*cos(xopt(2,:).');
% 
% 
% LFootVisualx2 = zeros(Nsim,1);
% LFootVisualy2 = zeros(Nsim,1);  %RfootPosxy(indices,2);
% HipVisualx2 =LFootVisualx2 + L*sin(xidle(1,:).');
% HipVisualy2 = LFootVisualy2 + L*cos(xidle(1,:).');
% RFootVisualx2 = HipVisualx2-L*sin(xidle(2,:).');
% RFootVisualy2 = HipVisualy2-L*cos(xidle(2,:).');
% 
% % myVideo = VideoWriter('Nonlinear_control_CMG'); %open video file
% % myVideo.FrameRate = 5;  %can adjust this, 5 - 10 works well for me
% % open(myVideo)
% 
% i = 1;
% walker = plot([LFootVisualx(i),HipVisualx(i),RFootVisualx(i)],[LFootVisualy(i),HipVisualy(i),RFootVisualy(i)],'o-','Color','r');
% walker2 = plot([LFootVisualx2(i),HipVisualx2(i),RFootVisualx2(i)],[LFootVisualy2(i),HipVisualy2(i),RFootVisualy2(i)],'o-','Color','k');
% plot(yref,0,'*')
% axis equal
% legend('ref', 'walker')
% drawnow
% 
% for i= 1:length(HipVisualx)
%     delete(walker)
%     delete(walker2)
%     walker = plot([LFootVisualx(i),HipVisualx(i),RFootVisualx(i)],[LFootVisualy(i),HipVisualy(i),RFootVisualy(i)],'o-','Color','r');
%     walker2 = plot([LFootVisualx2(i),HipVisualx2(i),RFootVisualx2(i)],[LFootVisualy2(i),HipVisualy2(i),RFootVisualy2(i)],'o-','Color','k');
%     axis equal
%     legend('ref', 'nonlin control', 'no control')
%     drawnow
%     pause(0.05)
% %     frame = getframe(gcf); %get frame
% %     writeVideo(myVideo, frame);
% end
% 
% % close(myVideo)

%% New method for kimp
kimpMethod = 1;
dt = 0.01;
CoMPos = simout.CoMPos.signals.values;
CoMVel = simout.CoMVel.signals.values;

%%% Estimate Limp

% Initialize loop
Limp = zeros(6,1);
for swingNumber = 6:12
    % Find the final index of each swing
    swingIndices = cell2mat(RSwingIndices(swingNumber));
    finalIndx = swingIndices(end);

    % Calculate the distance between the CoM and the ankle position at the
    % moment of impact, that distance is called Limp
    CoMPosEnd = CoMPos(finalIndx,1);
    AnklePosEnd = RAnklePosxy(finalIndx,1);
    Limp(swingNumber-5) = AnklePosEnd-CoMPosEnd;
end

% Evaluate whether Limp is approximately constant
Limp
averageLimp = mean(Limp)

%%% Estimate kimp
% Take a certain swing number and find the true swing time
swingNumber = 6;
swingIndices = cell2mat(RSwingIndices(swingNumber));
finalIndx = swingIndices(end);
Limp12 = Limp(end);
timpTrue = t(swingIndices(end))-t(swingIndices(1));
kimpTrue = round(timpTrue/dt);

% Initialize for loop
kimpEst = zeros(length(swingIndices),1);
kimpOrg = zeros(length(swingIndices),1);
for indx = 1:length(swingIndices)
    % Estimate remaining time using the ballistic assumption
    S = RAnklePosxy(finalIndx,1) - LAnklePosxy(finalIndx,1);
    if kimpMethod == 1
        Vel = CoMVel(swingIndices(indx),1); % Velocity of the centre of mass
        Pos = CoMPos(swingIndices(indx),1)-LAnklePosxy(swingIndices(indx),1);% Position of the center of mass
    elseif kimpMethod == 2
        Pos = L*sin(th1(swingIndices(indx)))+hipCorrection;
        PosPrevious = L*sin(th1(swingIndices(indx)-1))+hipCorrection;
        Vel = (Pos-PosPrevious)/dt;
    end
    tRemain = (S-averageLimp-Pos)/Vel;
    kRemain = round(tRemain/dt);
    kimpEst(indx) = kRemain + indx -1;
    x0 = x(swingIndices(indx),:);
    [timp, ximp, CP] = heelstrike(x0, params);
    kimpOrg(indx) = round(timp/dt)+ indx -1;
end 

% Evaluate the accuracy of the kimp estimation
kimpEst
kimpTrue

figure
plot(kimpOrg)
hold on
plot(kimpEst)
legend('original', 'new')

figure
plot(CoMVel(swingIndices,1))

%% Find cause of bad prediction
% % figure
% indices = cell2mat(RSwingIndices(7));
% xswing = x(indices,:)
% % plot(xswing)
% % legend('th1', 'th2', 'dth1','dth2')
% 
% xtest = xswing(25,:).'
% xtest(4) = xtest(4)+0.1
% kimp = 46-25;
% NMPC_mex(xtest,kimp,params);

