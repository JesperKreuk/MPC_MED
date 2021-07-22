clear all
clc
%% Load settings
setup_paths
model = 'CMGModelv2';

innerOptSettings = setInnerOptSettings(model,'resume','eval','targetVelocity', 1.2);
BodyMechParams;
ControlParams;  
MPCparams;
load(['Opt_gains' filesep 'v1.2ms.mat']);

%% Load model, create ground, set gains
load_system(model);
[groundX, groundZ, groundTheta] = generateGround('flat');

dt_sim = 0.001;
dt_visual = dt_sim;
animFrameRate = 10;

assignGainsSagittal;
assignGainsCoronal;
assignInit;

% %% Initialize extended kalman filter
% uInit = 0;
% xInit = zeros(4,1);
% EKF = trackingEKF(@(vars)myStateTransitionFcn(vars,uInit,params) ,@myMeasurementFcn,xInit);

%% Building rapid accelerator target
% warning('off')
% rtp = Simulink.BlockDiagram.buildRapidAcceleratorTarget(model);
% warning('on');
% 
% 
% [groundX, groundZ, groundTheta] = generateGround('flat',[],0,false);
% 
% paramSets = Simulink.BlockDiagram.modifyTunableParameters(rtp, ...
%     'groundZ',     groundZ, ...
%     'groundTheta', groundTheta);
% in = Simulink.SimulationInput(model);
% in = in.setModelParameter('TimeOut', 10*60);
% in = in.setModelParameter('SimulationMode', 'rapid', ...
%     'RapidAcceleratorUpToDateCheck', 'off');
% in = in.setModelParameter('StopTime', '30');
% in = in.setModelParameter('RapidAcceleratorParameterSets', paramSets);
% 

%% Simulate model

in = Simulink.SimulationInput(model);
simout = parsim(in, 'ShowProgress', true, 'TransferBaseWorkspaceVariables', 'on') ;
% 
% %% Plot data
% plotData(simout.GaitPhaseData, simout.stepTimes,...
%     'angularData',simout.angularData, 'musculoData',simout.musculoData, ...
%     'GRFData',simout.GRFData, 'jointTorquesData',simout.jointTorquesData, 'info',' ', 'saveFigure', false);
% %% Animate 
% animPost(simout.animData3D,'intact',true,'speed',1,'view','perspective',...
%                 'showFigure',true,'createVideo',false,'info',[num2str(innerOptSettings.targetVelocity) 'ms'],'saveLocation',innerOptSettings.optimizationDir);
%             

% %%
% m = matfile('simoutGreyEstData.mat', 'Writable', true);
% m.simout = simout