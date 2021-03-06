%{
This file performs a grey-box estimation of the nonlinear compass-gait
biped (CGB) model. The file uses data simulated by the 3D nms model of Song.
The parameters of the CGB model are optimized and validated on different
data sets from the simulation. Additionally, the model is linearized in two
different linearization points. It is evaluated how well the models fit the
data of the 3D nms model.

Author: Jesper Kreuk

License: The code is available for Academic or Non-Profit Organization 
Noncommercial research use only.
%}

%%
% This is the main file, clear all variables
clear all
close all
clc

% Add all folders to path
setup_paths

%% Settings
% Choose 1 to estimate the system, choose 0 to load the parameters
estimateSystem = 0; 

% Choose 1 to validate the system, choose 0 to skip this part
validateSystem = 0;

% Choose 1 to linearize system, new matlab functions for A, B and Baff will
% be made. Choose 0 to skip this part
linearizeSystem = 0;

%% Position tracking
% Different positions on the foot can be tracked, this section plots the
% trajectories of three options, the heel, anke and toe.

swingNumberID = 10; % this swing number will be used for grey-box estimation
dataset = 'greyboxData.mat'; % dataset used for grey-box estimation

% Find the trajectories for the different positions
[RdataxToe, swingPercentageData] = positionTracking(dataset,swingNumberID, 'toe', 0);
[RdataxAnkle] = positionTracking(dataset,swingNumberID, 'ankle', 1);
[RdataxHeel] = positionTracking(dataset,swingNumberID, 'heel', 0);

% Plot the trajectories
figure('DefaultAxesFontSize',11);

% Angle of the stance leg
subplot(221)
hold on
plot(swingPercentageData,RdataxToe(:,1),'k','LineWidth',1.2)
plot(swingPercentageData,RdataxAnkle(:,1),'b--','LineWidth',1.5)
plot(swingPercentageData,RdataxHeel(:,1),'r-.','LineWidth',1.5)
title('Stance leg')
ylabel('Angle in rad')
legend('Toe','Ankle','Heel', 'Location', 'northwest')

% Angle of the swing leg
subplot(222)
hold on
plot(swingPercentageData,RdataxToe(:,2),'k','LineWidth',1.2)
plot(swingPercentageData,RdataxAnkle(:,2),'b--','LineWidth',1.5)
plot(swingPercentageData,RdataxHeel(:,2),'r-.','LineWidth',1.5)
title('Swing leg')

% Angular velocity of the stance leg
subplot(223)
hold on
plot(swingPercentageData,RdataxToe(:,3),'k','LineWidth',1.2)
plot(swingPercentageData,RdataxAnkle(:,3),'b--','LineWidth',1.5)
plot(swingPercentageData,RdataxHeel(:,3),'r-.','LineWidth',1.5)
ylabel('Angular velocity in rad/s')
xlabel('Swing in %')

% Angular velocity of the swing leg
subplot(224)
hold on    
plot(swingPercentageData,RdataxToe(:,4),'k','LineWidth',1.2)
plot(swingPercentageData,RdataxAnkle(:,4),'b--','LineWidth',1.5)
plot(swingPercentageData,RdataxHeel(:,4),'r-.','LineWidth',1.5)
ylabel('Angular velocity in rad/s')
xlabel('Swing in %')

%% Estimate the parameters of the CGB model
% Run estimation or load parameters, depending on settings
if estimateSystem
    % Find the optimal model and parameters
    [parametersOpt, modelOpt] = greyboxEstimation(dataset, swingNumberID);
else
    % Load the optimal model and parameters
    load('CGBoptimalParameters.mat')
end

%% Validate model on other dataset
%%% Validation of prediction on the full dataset of the nonlinear model
swingNumberVal = 14;
if validateSystem
    greyboxValidation(dataset, swingNumberVal, modelOpt)
end

%% Simulate the nonlinear CGB model 
horizon = 60; % prediction horizon (dt = 1e-2, so 0.6s)

% Load data from simulation of the 3D model of Song
load(dataset,'simout');
loadGreyboxPositions;

% Find indices of swing and the hip correction
swingIndicesVal = findSwingIndices(dataset,swingNumberVal);
hipCorrection = calculateHipCorrection(dataset,swingNumberID);

% Calculate state x for swingnumber 
[Rdatax, ~, Ndata] = calculateSensorData(LAnklePosxy,...
            RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,swingIndicesVal);
x0 = Rdatax(1,:).';

% Load parameters
m = parametersOpt(1);
a = parametersOpt(2);
mH = parametersOpt(3);
phi1 = parametersOpt(4);
phi2 = parametersOpt(5);

dt = 1e-2; % Time step of discrete system
tspan = 0:dt:0.6;

% Simulate nonlinear CGB model
[~,xnonlin] = ode45(@(t,x)dynamics(t,x,m,a,mH,phi1,phi2), tspan, x0);
xnonlin = xnonlin.';

%% Linearize nonlinear CGB model (symbolic)
params = setupParamsStruct(parametersOpt);
if linearizeSystem
    % Linear system in continuous time
    [Asym,Bsym,Baffsym] = linearizeCGBModel(params);
    
    % Create matlab functions from symbolic expressions
    syms th1 th2 dth1 dth2 uCMG
    matlabFunction(Asym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Asymfun');
    matlabFunction(Bsym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Bsymfun');
    matlabFunction(Baffsym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Baffsymfun');
end

%% Simulate linear system for lambda = 0 from the initial position
%%% Linearize around the initial position
xeq = x0; % Equilibirum point
Ac1 = Asymfun(xeq,0);
Bc1 = Bsymfun(xeq,0);
Baffc1 = Baffsymfun(xeq,0);

% Simulate linear CGB model with lambda = 0
xlin = x0;
for i = 1:horizon
    xdot(:,i) =Ac1*xlin(:,i)+Baffc1;
    xlin(:,i+1) = xlin(:,i)+xdot(:,i)*dt;
end

%% Simulate linear system for lambda = 0.5 from the initial position
% Find the linearization point and linearize
lambda = 0.5;
xeq2 = findLinearizationPoint(xnonlin, x0, lambda);

Ac2 = Asymfun(xeq2,0);
Bc2 = Bsymfun(xeq2,0);
Baffc2 = Baffsymfun(xeq2,0);

% Simulate linear CGB model with lambda = 0.5
xlin2 = x0;
for i = 1:horizon
    xdot2(:,i) =Ac2*xlin2(:,i)+Baffc2;
    xlin2(:,i+1) = xlin2(:,i)+xdot2(:,i)*dt;
end

%% At halfway point of the swing
x0half = Rdatax(round(Ndata/2),:).';
Nmodel = round(Ndata/10)+1;

[~,xnonlinHalf] = ode45(@(t,x) dynamics(t,x,m,a,mH,phi1,phi2), tspan, x0half);
xnonlinHalf = xnonlinHalf.';
% Dynamics in the linearization point
Ac3 = Asymfun(x0half,0);
Bc3 = Bsymfun(x0half,0);
Baffc3 = Baffsymfun(x0half,0);

xlin3 = x0half;
for i = 1:horizon
    xdot3(:,i) = Ac3*xlin3(:,i)+Baffc3;
    xlin3(:,i+1) = xlin3(:,i)+xdot3(:,i)*dt;
end
cutSwingPercentageData = mapfun(round(Nmodel/2):round(Nmodel/2)+horizon, 1, Nmodel,0,100);
Ncut = 25;

% linearization point 2
xeqHalf = findLinearizationPoint(xnonlinHalf, x0half, lambda);

A4 = Asymfun(xeqHalf,0);
B4 = Bsymfun(xeqHalf,0);
Baff4 = Baffsymfun(xeqHalf,0);

xlin4 = x0half;
for i = 1:horizon
    xdot4(:,i) = A4*xlin4(:,i)+Baff4;
    xlin4(:,i+1) = xlin4(:,i)+xdot4(:,i)*dt;
end

%% Visualize trajectories of nonlinear and linear models 
colors; % Load colors for plot
figure('DefaultAxesFontSize',11)
swingPercentageData = linspace(0,100,Ndata);
swingPercentageModel = linspace(0,100,Nmodel);

% This subplot is for theta 1 simulated from 0% swing
subplot(221)
hold on
plot(swingPercentageData,Rdatax(:,1),'g','LineWidth',1)
plot(swingPercentageModel,xlin(1,1:Nmodel).',':','color',Color1(3,:),'LineWidth',2);
plot(swingPercentageModel,xlin2(1,1:Nmodel).','-.','color',Color1(4,:),'LineWidth',1.7);
plot(swingPercentageModel,xnonlin(1,1:Nmodel),'--','color',Color1(5,:),'LineWidth',1.3);
axis([0,100,-0.3,0.4])
ylabel('\theta_1 in rad')
title('Start at 0% swing')
legend('Data','Lin \lambda = 0','Lin \lambda = 0.5','Nonlin','location','northwest')

% This subplot is for theta 1 simulated from 50% swing
subplot(222)
hold on
plot(swingPercentageData,Rdatax(:,1),'g','LineWidth',1)
plot(cutSwingPercentageData(1:Ncut),xlin3(1,1:Ncut).',':','color',Color1(3,:),'LineWidth',2);
plot(cutSwingPercentageData(1:Ncut),xlin4(1,1:Ncut).','-.','color',Color1(4,:),'LineWidth',1.7);
plot(cutSwingPercentageData(1:Ncut),xnonlinHalf(1,1:Ncut),'--','color',Color1(5,:),'LineWidth',1.3);
axis([0,100,-0.3,0.4])
title('Start at 50% swing')

% This subplot is for theta 2 simulated from 0% swing
subplot(223)
hold on
plot(swingPercentageData,Rdatax(:,2),'g','LineWidth',1)
plot(swingPercentageModel,xlin(2,1:Nmodel).',':','color',Color1(3,:),'LineWidth',2);
plot(swingPercentageModel,xlin2(2,1:Nmodel).','-.','color',Color1(4,:),'LineWidth',1.7);
plot(swingPercentageModel,xnonlin(2,1:Nmodel),'--','color',Color1(5,:),'LineWidth',1.3);
axis([0,100,-0.8,0.4])
ylabel('\theta_2 in rad')
xlabel('Swing in %')

% This subplot is for theta 2 simulated from 50% swing
subplot(224)
hold on 
plot(swingPercentageData,Rdatax(:,2),'g','LineWidth',1)
plot(cutSwingPercentageData(1:Ncut),xlin3(2,1:Ncut).',':','color',Color1(3,:),'LineWidth',2);
plot(cutSwingPercentageData(1:Ncut),xlin4(2,1:Ncut).','-.','color',Color1(4,:),'LineWidth',1.7);
plot(cutSwingPercentageData(1:Ncut),xnonlinHalf(2,1:Ncut),'--','color',Color1(5,:),'LineWidth',1.3);
axis([0,100,-0.8,0.4])
xlabel('Swing in %')

%% Table for thesis report
cost_func = 'NRMSE';
names = {'lin 1';'lin 2'; 'nonlin'};
subplot1 = zeros(3,1);
subplot2 = zeros(3,1);
subplot3 = zeros(3,1);
subplot4 = zeros(3,1);

% Subplot 1
ydes = downsample(Rdatax(:,1),10);
y = xlin(1,1:Nmodel).';
subplot1(1) = goodnessOfFit(y,ydes,cost_func);
y = xlin2(1,1:Nmodel).';
subplot1(2) = goodnessOfFit(y,ydes,cost_func); 
y = xnonlin(1,1:Nmodel).';
subplot1(3) = goodnessOfFit(y,ydes,cost_func); 

% Subplot 2
ydes = downsample(Rdatax(round(Ndata/2):end,1),10);
y = xlin3(1,1:Ncut).';
subplot2(1) = goodnessOfFit(y,ydes,cost_func);
y = xlin4(1,1:Ncut).';
subplot2(2) = goodnessOfFit(y,ydes,cost_func); 
y = xnonlinHalf(1,1:Ncut).';
subplot2(3) = goodnessOfFit(y,ydes,cost_func); 

% Subplot 3
ydes = downsample(Rdatax(:,2),10);
y = xlin(2,1:Nmodel).';
subplot3(1) = goodnessOfFit(y,ydes,cost_func);
y = xlin2(2,1:Nmodel).';
subplot3(2) = goodnessOfFit(y,ydes,cost_func); 
y = xnonlin(2,1:Nmodel).';
subplot3(3) = goodnessOfFit(y,ydes,cost_func); 

% Subplot 4
ydes = downsample(Rdatax(round(Ndata/2):end,2),10);
y = xlin3(2,1:Ncut).';
subplot4(1) = goodnessOfFit(y,ydes,cost_func);
y = xlin4(2,1:Ncut).';
subplot4(2) = goodnessOfFit(y,ydes,cost_func); 
y = xnonlinHalf(2,1:Ncut).';
subplot4(3) = goodnessOfFit(y,ydes,cost_func); 

T = table(names,subplot1,subplot2,subplot3,subplot4);
disp(T)


%% Optimize lambda
[J3Data, J3Model] = findAlphaMatrices(dataset,swingNumberVal, params);

[timp, ximp] = heelstrike(x0,params); % Find when heelstrike occurs
kimp = round(timp/dt);

lambdaRange = 0:0.01:1;

[~,I] = max(J3Data,[],1);
for i = 1:kimp
    indx = I(i);
    lambdaopt1(i) = lambdaRange(indx);
end
[~,I] = max(J3Model,[],1);
lambdaopt2 = [];
for i = 1:kimp
    indx = I(i);
    lambdaopt2(i) = lambdaRange(indx);
end
swingPercentagelambda = linspace(0,100,kimp);


figure('DefaultAxesFontSize',11)
plot(swingPercentagelambda,lambdaopt1,'--', 'Linewidth', 2)
hold on
plot(swingPercentagelambda,lambdaopt2,'-.','Linewidth', 2)
xlabel('Swing in %')
ylabel('\lambda_o_p_t')
legend('Nonlinear model','Data set', 'location','northwest')

%% Compare model Timmers and Song
load('greyboxDataTimmers.mat')

% Time
t = simout.greyEstData.time;

% Extract relevant locations
HATPosxy = simout.greyEstData.signals.values(:,1:2);
LToePosxy = simout.greyEstData.signals.values(:,3:4);
LAnklePosxy = simout.greyEstData.signals.values(:,5:6);
LHeelPosxy = simout.greyEstData.signals.values(:,7:8);
LKneePosxy = simout.greyEstData.signals.values(:,9:10);
LHipPosxy = simout.greyEstData.signals.values(:,11:12);
RToePosxy = simout.greyEstData.signals.values(:,13:14);
RAnklePosxy = simout.greyEstData.signals.values(:,15:16);
RHeelPosxy = simout.greyEstData.signals.values(:,17:18);
RKneePosxy = simout.greyEstData.signals.values(:,19:20);
RHipPosxy = simout.greyEstData.signals.values(:,21:22);

% This constant is calculated in calculateSensorData.m
dtTimmers = 1e-3;

GaitPhaseData       = simout.GaitPhaseData;   
rightLegState       = GaitPhaseData.signals.values(:,2);

% Data valid while the state is 3 and 4, that is the swinging leg
RswingIdx = find(rightLegState == 3 | rightLegState == 4);
RswingEndIdx = find(diff(RswingIdx)>1);


RSwingIndices = cell(length(RswingEndIdx),1);
RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
for i = 1:length(RswingEndIdx)-1
    RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
end

indices = cell2mat(RSwingIndices(swingNumberVal));

Ndata = length(indices);
swingpercentagedataNathan = linspace(0,100,Ndata);
% Sensor placement
RfootPosxy = RAnklePosxy;
LfootPosxy = LAnklePosxy;

% Calculate the hip correction, which is the displacement in x position of
% the hip to make the compass-gait biped hit the ground at the same time as
% the model.
RFootEnd = RfootPosxy(indices(end),1);
LFootEnd = LfootPosxy(indices(end),1);
RHipPosEnd = RHipPosxy(indices(end),1);
LHipPosEnd = LHipPosxy(indices(end),1);
hipPosEnd = (LHipPosEnd+RHipPosEnd)/2;
hipCorrection = hipPosEnd-(LFootEnd+RFootEnd)/2;

HipPosxy = (LHipPosxy + RHipPosxy)/2-hipCorrection;

th1 = asin((HipPosxy(:,1)-LfootPosxy(:,1))/params.L);
th2 = asin((HipPosxy(:,1)-RfootPosxy(:,1))/params.L);

% Calculate dth1 and dth2
dth1 = gradient(th1,dtTimmers);
dth2 = gradient(th2,dtTimmers);

% Define x and u
x = [th1,th2,dth1,dth2];
u = cos(x(:,1:2));

% Cut data
tdata = t(indices);
LdataxNathan = x(indices,:);


figure('DefaultAxesFontSize',11); hold on

subplot(221)
hold on
plot(swingPercentageData,Rdatax(:,1),'LineWidth',1.5)
plot(swingpercentagedataNathan,LdataxNathan(:,1),'-.','LineWidth',1.5)
title('Stance leg')
ylabel('Angle in rad')
legend('Song','Nathan','Location', 'northwest')

subplot(222)
hold on
plot(swingPercentageData,Rdatax(:,2),'LineWidth',1.5)
plot(swingpercentagedataNathan,LdataxNathan(:,2),'-.','LineWidth',1.5)
title('Swing leg')

subplot(223)
hold on
plot(swingPercentageData,Rdatax(:,3),'LineWidth',1.5)
plot(swingpercentagedataNathan,LdataxNathan(:,3),'-.','LineWidth',1.5)

ylabel('Angular velocity in rad/s')
xlabel('Swing in %')

subplot(224)
hold on    
plot(swingPercentageData,Rdatax(:,4),'LineWidth',1.5)
plot(swingpercentagedataNathan,LdataxNathan(:,4),'-.','LineWidth',1.5)
ylabel('Angular velocity in rad/s')
xlabel('Swing in %')


%% Compare linear model with nonlinear model in an interactive plot  
% figure options
f = figure;
ax = axes('Parent',f,'position',[0.13 0.29  0.77 0.64]);
xlabel('Swing in %')
ylabel('Angle in rad')
slider = uicontrol('Parent',f,'Style','slider','Position',[100,50,620,23],...
              'value',1, 'min',1, 'max',Ndata);
slider.UserData = Ndata;
slider.Callback = @plotLines;
set(gcf,'position',[50,50,800,600]);

% plot data
hold on
plot(swingPercentageData,Rdatax(:,1:2),'g','LineWidth',1.5)
nonlinPrediction = plot(swingPercentageModel,xnonlin(1:2,1:Nmodel),'k','LineWidth',1.3);
linPrediction = plot(swingPercentageModel,xlin(1:2,1:Nmodel).','b--','LineWidth',1.7);
linPrediction2 = plot(swingPercentageModel,xlin2(1:2,1:Nmodel).','r--','LineWidth',1.7);
legend('\theta_1 data','\theta_2 data','\theta_1 nonlin','\theta_2 nonlin', ...
    '\theta_1 lin \lambda = 0','\theta_2 lin \lambda = 0', ...
    '\theta_1 lin \lambda = 0.5','\theta_2 lin \lambda = 0.5','location','southwest')
axis([0,100,-0.8,0.5])

% save data to use in the plotLines callback function
setappdata(f,'horizon',horizon);
setappdata(f,'Rdatax',Rdatax);
setappdata(f,'dt',dt);
setappdata(f,'Ndata',Ndata);
setappdata(f,'Nmodel',Nmodel);
setappdata(f,'linPrediction',linPrediction);
setappdata(f,'linPrediction2',linPrediction2);
setappdata(f,'nonlinPrediction',nonlinPrediction);
setappdata(f,'f',f);
setappdata(f,'ParametersOpt',parametersOpt);

