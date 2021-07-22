%{
This MATLAB file is able to simulate the 3D walking model of Song with
the MLD controller or the nonlinear control method.
%}

clear all
close all
clc


% %% Setup
setup_paths
dt = 1e-2; % Step time of the controller

estimateParams = 0;
runMLD = 1;
runNonlin = 1;

dataset = 'simoutGreyEstDataSong.mat'; % dataset used for identification
 
%% Find/load parameters of the compass walker

% Compass walker
if estimateParams
    swingNumber = 15; % Use the dataset of the 10th swing for identification
    
    % Find the optimal model and parameters
    [modelOpt, ParametersOpt] = greyboxEstimation(dataset, swingNumber)
else
    load('Optimal_params_30percentSong.mat')
end
Qy = 1e8;
yref = 0.7;
params = setupParamsStruct(ParametersOpt, Qy, yref, dt);


%% Simulate MLD control
% Linear system in continuous time
[Asym,Bsym,Baffsym] = linearizeCWModel(params);

% syms th1 th2 dth1 dth2 uCMG
% matlabFunction(Asym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Asymfun');
% matlabFunction(Bsym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Bsymfun');
% matlabFunction(Baffsym,'Vars',{[th1;th2;dth1;dth2],uCMG},'File','Baffsymfun');

controlStart = 5;

% model = 'nms_3Dmodel_nonlin';
model = 'nms_3Dmodel_mld';
runScripts;
simout = sim(model);
%% Evaluate the step length 
% This constant is calculated in calculateSensorData.m
hipCorrection = params.hipCorrection;
L = params.L;

HATPosxy = simout.jointData.signals.values(:,1:2);
LAnklePosxy = simout.jointData.signals.values(:,3:4);
LHipPosxy = simout.jointData.signals.values(:,5:6);
RAnklePosxy = simout.jointData.signals.values(:,7:8);
RHipPosxy = simout.jointData.signals.values(:,9:10);

RswingIdx = find(simout.RSwing.signals.values>0);
RswingEndIdx = find(diff(RswingIdx)>1);

RSwingIndices = cell(length(RswingEndIdx),1);
RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
for i = 1:length(RswingEndIdx)-1
    RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
end

swingNumber = 6;
indices = cell2mat(RSwingIndices(swingNumber));

HipPosxy = (LHipPosxy + RHipPosxy)/2-hipCorrection;

% Theta 1 is always the left healthy and theta 2 is always the right prosthetic leg
th1 = asin((HipPosxy(:,1)-LAnklePosxy(:,1))/L);
th2 = asin((HipPosxy(:,1)-RAnklePosxy(:,1))/L);

% Calculate dth1 and dth2
dth1 = gradient(th1,dt);
dth2 = gradient(th2,dt);

% Define x and u
x = [th1,th2,dth1,dth2];

finalIndx = indices(end);
th1end = x(finalIndx,1);
th2end = x(finalIndx,2);
stepLength =  L*sin(th1end)-L*sin(th2end)

%% Evaluate control
Indx = indices(1:end-2);
torqueNonlin = simout.controlData.signals.values(Indx,3);
swingPercentNonlin = linspace(0,100,length(indices(1:end-2)));
figure
plot(swingPercentNonlin,torqueNonlin,'k','LineWidth',2)
hold on
plot([0,100],[0,0],'r--','LineWidth',1)
plot([0,100],[5,5],'r--','LineWidth',1)
xlabel('Swing in percent')
ylabel('Torque in Nm')
axis([0,100,-1,6])
