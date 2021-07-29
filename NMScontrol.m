%{
This file finds the optimal constant torque of the momentum exchange device
to reach a given step length. Different step references and disturbances
are tested to challenge the controller. Additionally, the relation between
the cost function to optimize and the constant torque is plotted. 

Author: Jesper Kreuk

License: The code is available for Academic or Non-Profit Organization 
Noncommercial research use only.
%}

% This is the main file, clear all variables
clear all
close all
clc

% Add all folders to path
setup_paths

%% Settings
% Setting the test variables to 1 makes the optimization run, this can take
% a while.
disturbanceTest = 0;
refTest = 0;
%% Parameters
% Load parameters
nms_model_MechInit;
nms_model_ControlInit;
initState;

% The step time is set to inherent, this way the time is sampled at heel strike
dt_visual = -1; 

umax = 5; % Assume a maximum torque of 5 Nm

%% Optimization settings*

model = 'nms_model_Push';

uConstant = 0;

% Vector with torque disturbances 
% [Lfoot; Lshank; Lthigh; HAT; Rthigh; Rshank; HATpush]
disturbances = zeros(7,1);
distStart = 2.29;
distStop = 2.3;
controlStart = 2.3;

distAmp = -50:5:50;
distAmp(11) = [];
%% Run without distrubances
warning off
simout = sim(model,'SrcWorkspace','current');
warning on
stepLength =  simout.RFootPos.signals.values(end,1)-simout.LFootPos.signals.values(end,1)


%% Optimize input for different torque disturbances
counter = 0;
yref = 0.8;
if disturbanceTest
    for i = 4:6
        for j = 1:length(distAmp)
            counter = counter + 1;
            disturbances = zeros(7,1);
            disturbances(i) = distAmp(j);
            [uopt, Jopt, uPoints, JPoints,flag] = findOptimalControl(umax, yref, model, disturbances, distStart, distStop, controlStart);
            alluopt{counter} = uopt;
            allJ{counter} = Jopt;
            alluPoints{counter} = uPoints;
            allJPoints{counter} = JPoints;
            allDisturbances{counter} = disturbances;
            allflags{counter} = flag;
        end
    end
end

%% Optimize input for different references
if refTest
    disturbances = zeros(7,1);
    references = 0.75:0.005:1.05;
    for i = 1:length(references)
        yref = references(i)
        disturbances = zeros(7,1);
        [uopt, Jopt, uPoints, JPoints] = findOptimalControl(umax, yref, model, disturbances, distStart, distStop, controlStart);
            alluopt{i} = uopt;
            allJopt{i} = Jopt;
            alluPoints{i} = uPoints;
            allJPoints{i} = JPoints;
    end
end

%% Plot optimal control different references
figure('DefaultAxesFontSize',11); hold on
h = zeros(1,4);
colors;
h(1) = plot([0,100],[0,0],'color',Color2(3,:),'LineWidth',1);
h(2) = plot([0,100],[5,5],'color',Color2(3,:),'LineWidth',1);

load('refOptShank')
h(3) = plot(references(1:end),cell2mat(alluopt(1:end)),'k--','LineWidth',2.5);

load('refOptThigh')
h(4) = plot(references(1:end-20),cell2mat(alluopt(1:end-20)),'b-.','LineWidth',2.5);

xlabel('Reference in m')
ylabel('Torque in Nm')
axis([references(1),references(end-20),-1,6])
legend([h(3:4),h(1)],'Shank placement', 'Thigh placement','Bounds','location','northwest')

%% Plot optimal control different disturbances
figure('DefaultAxesFontSize',11); hold on

load('torqueTest.mat')
for i = 1:3
    if i == 1
    plot(distAmp(3:end-2), cell2mat(alluopt(20*(i-1)+3:20*i-2)),'--','LineWidth', 2.5)
    elseif i == 2
    plot(distAmp(3:end-2), cell2mat(alluopt(20*(i-1)+3:20*i-2)),'-.','LineWidth', 2.5)
    elseif i == 3
    plot(distAmp(3:end-2), cell2mat(alluopt(20*(i-1)+3:20*i-2)),':','LineWidth', 2.5)
    end
end
legend('HAT','Thigh', 'Shank')
xlabel('Disturbance amplitude in Nm')
ylabel('Optimal actuator torque in Nm')

load('pushTest.mat')
figure('DefaultAxesFontSize',11); hold on
for i = 1:3
    if i == 1
    plot(distAmp, cell2mat(alluopt(20*(i-1)+1:20*i)),'--','LineWidth', 2.5)
    elseif i == 2
    plot(distAmp, cell2mat(alluopt(20*(i-1)+1:20*i)),'-.','LineWidth', 2.5)
    elseif i == 3
    plot(distAmp, cell2mat(alluopt(20*(i-1)+1:20*i)),':','LineWidth', 2.5)
    end
end
legend('HAT','Thigh', 'Shank')
xlabel('Disturbance amplitude in N')
ylabel('Optimal actuator torque in Nm')

%% Plot cost function J2 for different ucont
load('200gridpoints.mat')

figure('DefaultAxesFontSize',11); hold on
plot(ugrid,Jgrid,'k','linewidth',2)
xlabel('{\it u}_c_o_n in Nm')
ylabel('{\it J}_2 in m^2')
