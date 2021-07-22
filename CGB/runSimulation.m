%{
This MATLAB file is able to simulate the 3D walking model of Song with
the MLD controller or the nonlinear control method.
%}

clear all
close all
clc

%% Setup
setup_paths
dt = 1e-2; % Step time of the controller

estimateParams = 0;
runMLD = 1;
runNonlin = 1;

dataset = 'simoutGreyEstData.mat'; % dataset used for identification
 
%% Find/load parameters of the compass walker

% Compass walker
if estimateParams
    swingNumber = 10; % Use the dataset of the 10th swing for identification
    
    % Find the optimal model and parameters
    [modelOpt, ParametersOpt] = greyboxEstimation(dataset, swingNumber)
else
    load('Optimal_params_30percent.mat')
end
Qy = 1e8;
yref = 0.65;
params = setupParamsStruct(ParametersOpt, Qy, yref, dt);


%% Simulate MLD control
% Linear system in continuous time
[Asym,Bsym,Baffsym] = linearizeCWModel(params);

syms th1 th2 dth1 dth2 uCMG
matlabFunction(Asym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Asymfun');
matlabFunction(Bsym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Bsymfun');
matlabFunction(Baffsym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Baffsymfun');


model = 'nms_model_mld2019a';
innerOptSettings = setInnerOptSettings(model,'resume','eval','targetVelocity', 1.2);
load(['Opt_gains' filesep 'v1.2ms.mat']);
[groundX, groundZ, groundTheta] = generateGround('flat');
BodyMechParams;
ControlParams;
assignGainsSagittal;
assignGainsCoronal;
assignInit;
% in = Simulink.SimulationInput(model);
% % set_param('nms_model_mld2019a','SaveFinalState','on','FinalStateName',...
% % 'myOperPoint','SaveOperatingPoint','on');
% simout_mld = parsim(in, 'ShowProgress', true, 'TransferBaseWorkspaceVariables', 'on') ;
% myOperPoint = simout_mld.myOperPoint;

%% Continue from operating point
% op1 = simscape.op.create(simout_mld.testlog, 2)
% t = simscape.op.Target(-40, 'deg', 'High')
% op2 = set(op1,'Body Mechanics Layer/Left Ankle joint/Rz/q',t)
% sim('nms_model_mld2019a')

%% Simulate nonlin control
buildMexFile(params);
model = 'nms_model_nonlin';
innerOptSettings = setInnerOptSettings(model,'resume','eval','targetVelocity', 1.2);
load(['Opt_gains' filesep 'v1.2ms.mat']);
[groundX, groundZ, groundTheta] = generateGround('flat');
BodyMechParams;
ControlParams;
assignGainsSagittal;
assignGainsCoronal;
assignInit;
in = Simulink.SimulationInput(model);
simout_nonlin = parsim(in, 'ShowProgress', true, 'TransferBaseWorkspaceVariables', 'on') ;
