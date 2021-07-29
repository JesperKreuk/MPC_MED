%{
This MATLAB file is able to simulate the 3D walking model of Song with
the MLD controller or the nonlinear control method.
%}

%%
% This is the main file, clear all variables
clear all
close all
clc

% Setup paths
setup_paths


%% Settings
Qy = 1e8;
yref = 0.70;
controlStart = 5;

%% Load data
dt = 1e-2; % Step time of the controller
load('CGBoptimalParameters.mat')
controlParams.Qy = Qy;
controlParams.yref = yref;
params = setupParamsStruct(parametersOpt, dt, controlParams);


%% Simulate MLD control
swingNumber = 6;
runScripts;

alpha = 0;
model = 'nms_3Dmodel_mld';
simoutMLD0 = sim(model);
[stepLengthMLD0, indicesMLD0] = findStepLength(simoutMLD0, swingNumber);

alpha = 0.5;
simoutMLD50 = sim(model);
[stepLengthMLD50,indicesMLD50] = findStepLength(simoutMLD50,swingNumber);

%% Simulate NMPC control    
model = 'nms_3Dmodel_nonlin';
simoutNMPC = sim(model);
[stepLengthNMPC, indicesNMPC] = findStepLength(simoutNMPC, swingNumber);

%% Evaluate control
Indx = indicesMLD0(1:end-2);
torqueMLD0 = simoutMLD0.controlData.signals.values(Indx,3);
swingPercentMLD0 = linspace(0,100,length(indicesMLD0(1:end-2)));
Indx = indicesMLD50(1:end-2);
torqueMLD50 = simoutMLD50.controlData.signals.values(Indx,3);
swingPercentMLD50 = linspace(0,100,length(indicesMLD50(1:end-2)));
Indx = indicesNMPC(1:end-2);
torqueNonlin = simoutNMPC.controlData.signals.values(Indx,3);
swingPercentNonlin = linspace(0,100,length(indicesNMPC(1:end-2)));

colors;
figure('DefaultAxesFontSize',11)
hold on
h = zeros(1,5);
h(1) = plot([0,100],[0,0],'color',Color2(3,:),'LineWidth',1);
h(2) = plot([0,100],[5,5],'color',Color2(3,:),'LineWidth',1);
h(3) = plot(swingPercentMLD0,torqueMLD0,':','LineWidth', 2.5,'color',Color1(3,:));
h(4) = plot(swingPercentMLD50,torqueMLD50,'-.','LineWidth',2.5,'color',Color1(4,:));
h(5) = plot(swingPercentNonlin,torqueNonlin,'--','LineWidth',2.5,'color',Color1(5,:));
legend([h(3:5),h(2)], 'Lin \alpha = 0','Lin \alpha = 0.5','Nonlin','Bounds', 'location','southeast')


xlabel('Swing in %')
ylabel('Torque in Nm')
axis([0,100,-1,6])

