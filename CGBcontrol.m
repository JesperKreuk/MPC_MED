%{
This file simulates the 3D walking model of Song with
the MLD controllers and the nonlinear control method based on the compass-
gait biped (CGB) model. The resulting control actions are plotted.

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
Qy = 1e8;           % Weight on the Qy(y-yref)^2 with y being the ankle to ankle distance
yref = 0.70;        % The ankle to ankle reference
controlStart = 5;   % The controller is activated from this point in time

%% Load data
dt = 1e-2; % Step time of the controller
load('CGBoptimalParameters.mat')

% Setup parameter structure
controlParams.Qy = Qy;
controlParams.yref = yref;
params = setupParamsStruct(parametersOpt, dt, controlParams);


%% Simulate MLD control
swingNumber = 6; % This swing is when the control starts
runScripts;     % Setup variables for simulation of the 3D NMS model 

% Linearization at the state of the current time step
alpha = 0;
model = 'nms_3Dmodel_mld';
simoutMLD0 = sim(model);
[stepLengthMLD0, indicesMLD0] = findStepLength(simoutMLD0, swingNumber);

% Linearization in the middle of the current state and heel strike state
alpha = 0.5;
simoutMLD50 = sim(model);
[stepLengthMLD50,indicesMLD50] = findStepLength(simoutMLD50,swingNumber);

%% Simulate NMPC control    
model = 'nms_3Dmodel_nonlin';
simoutNMPC = sim(model);
[stepLengthNMPC, indicesNMPC] = findStepLength(simoutNMPC, swingNumber);

%% Evaluate control
% Extract the torques used by the controllers, normalise time
Indx = indicesMLD0(1:end-2);
torqueMLD0 = simoutMLD0.controlData.signals.values(Indx,3);
swingPercentMLD0 = linspace(0,100,length(indicesMLD0(1:end-2)));
Indx = indicesMLD50(1:end-2);
torqueMLD50 = simoutMLD50.controlData.signals.values(Indx,3);
swingPercentMLD50 = linspace(0,100,length(indicesMLD50(1:end-2)));
Indx = indicesNMPC(1:end-2);
torqueNonlin = simoutNMPC.controlData.signals.values(Indx,3);
swingPercentNonlin = linspace(0,100,length(indicesNMPC(1:end-2)));

colors; % Load colors used in the plot

% Plot the figure with the optimal torque against normalised time
figure('DefaultAxesFontSize',11)
hold on
h = zeros(1,5);
h(1) = plot([0,100],[0,0],'color',Color2(3,:),'LineWidth',1); % Lower bound
h(2) = plot([0,100],[5,5],'color',Color2(3,:),'LineWidth',1); % Upper bound
h(3) = plot(swingPercentMLD0,torqueMLD0,':','LineWidth', 2.5,'color',Color1(3,:));
h(4) = plot(swingPercentMLD50,torqueMLD50,'-.','LineWidth',2.5,'color',Color1(4,:));
h(5) = plot(swingPercentNonlin,torqueNonlin,'--','LineWidth',2.5,'color',Color1(5,:));
legend([h(3:5),h(2)], 'Lin \lambda = 0','Lin \lambda = 0.5','Nonlin','Bounds', 'location','southeast')
xlabel('Swing in %')
ylabel('Torque in Nm')
axis([0,100,-1,6])

