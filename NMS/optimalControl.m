clear all
close all
clc
%% Parameters
nms_model_MechInit;
nms_model_ControlInit;

dt_visual = 0.01;
%% Optimization settings
yref = 0.69;
lb = 2;
ub = 3;

model = 'nms_model_modified';

%% Optimization fmincon
% cInit = 0.6154;
% opt = optimoptions('fmincon', 'FiniteDifferenceStepSize', 1e-5);
% uopt = fmincon(fun,cInit,[],[],[],[],lb,ub,[],opt);

%% Optimization genetic algorithm
fun = @(uConstant)costFunction(uConstant, yref, model, dt_visual);
opts = optimoptions('ga','FitnessLimit',1e-08,'PopulationSize',10);

tic
uConstantOpt = ga(fun,1,[],[],[],[],lb,ub,[],opts);
toc
%% Optimization particle swarm
% options = optimoptions('particleswarm', 'ObjectiveLimit' , 1e-9);
% tic
% uConstantOpt = particleswarm(fun,1,lb,ub, options)
% toc
%% Simulation of optimal result
uConstant = uConstantOpt;
% uConstant =  2;

tic
simout = sim(model,'SrcWorkspace','current');
toc

stepLength =  simout.RFootPos.signals.values(end,1)-simout.LFootPos.signals.values(end,1)
J = (stepLength-yref)^2

%% Visualize result
load('nocontrol3.mat')
figure
plot(AngRAnk.signals.values(:,1)./pi*180)

% No control
LFootPosxy = LFootPos.signals.values(:,1:2);
LHipPosxy = xyzLHip.signals.values(:,1:2);
RHipPosxy = xyzRHip.signals.values(:,1:2);
HipPosxy = (LHipPosxy + RHipPosxy)./2;
RFootPosxy = RFootPos.signals.values(:,1:2);

% With control
LFootPosxyC = simout.LFootPos.signals.values(:,1:2);
LHipPosxyC = simout.xyzLHip.signals.values(:,1:2);
RHipPosxyC = simout.xyzRHip.signals.values(:,1:2);
HipPosxyC = (LHipPosxy + RHipPosxy)./2;
RFootPosxyC = simout.RFootPos.signals.values(:,1:2);

LFootPosxy-LFootPosxyC

N = length(LFootPosxyC);
i = 1;

figure
hold on
walker = plot([LFootPosxy(i,1),HipPosxy(i,1),RFootPosxy(i,1)],[LFootPosxy(i,2),HipPosxy(i,2),RFootPosxy(i,2)],'k-o');
walkerC = plot([LFootPosxyC(i,1),HipPosxyC(i,1),RFootPosxyC(i,1)],[LFootPosxyC(i,2),HipPosxyC(i,2),RFootPosxyC(i,2)],'k-o');
refPoint = plot(yref+LFootPosxy(i,1),0,'b*');

drawnow       
for i = 2:N 
    delete(refPoint)
    delete(walker)
    delete(walkerC)
    
    walker = plot([LFootPosxy(i,1),HipPosxy(i,1),RFootPosxy(i,1)],[LFootPosxy(i,2),HipPosxy(i,2),RFootPosxy(i,2)],'k-o');
    walkerC = plot([LFootPosxyC(i,1),HipPosxyC(i,1),RFootPosxyC(i,1)],[LFootPosxyC(i,2),HipPosxyC(i,2),RFootPosxyC(i,2)],'r-o');
    refPoint = plot(0.6921+LFootPosxy(i,1),0,'b*');
    
    axis equal
    ylim([0 1])
    legend('No control','Control','Reference')
    drawnow
end