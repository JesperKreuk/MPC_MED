
clear all
close all
clc


% %% Setup
setup_paths
dt = 1e-2; % Step time of the controller

estimateParams = 0;
swingNumber = 6; 

dataset = 'simoutGreyEstDataSong.mat'; % dataset used for identification
load(dataset)

%% load parameters of the compass walker
load('Optimal_params_30percentSong.mat')
Qy = 1e8;
yref = 0.7;
params = setupParamsStruct(ParametersOpt, Qy, yref, dt);

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
hipCorrection = 0.0816;



%% Find heelstrike
RswingIdx = find(simout.RSwing.signals.values>0);
RswingEndIdx = find(diff(RswingIdx)>1);

RSwingIndices = cell(length(RswingEndIdx),1);
RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
for i = 1:length(RswingEndIdx)-1
    RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
end


allIndices = cell2mat(RSwingIndices(swingNumber));
Ndata = length(allIndices);
%% Extract data from swingnumber (x,u)
[Ldatax, Ldatau, ~] = calculateSensorData(LAnklePosxy,...
            RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,allIndices);
data = iddata(Ldatax(:,1:2), Ldatau, dt');

x0 = [Ldatax(1,:).';1];
t = 6;
counter = 0;
y = mldController(x0,t,counter)

%%
y = nonlinController(x0,t,counter)
