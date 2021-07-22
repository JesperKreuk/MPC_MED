clear all
close all
clc

%% Choose sensor
sensor_stance = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively
sensor_hip = 'hipcor'; % Choose 'hip','hipcor', 'hat', for the hip, hip with correction or the hat respectively
sensor_swing = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively

%% Choose data set
swingNumber = 15;
%% Setup paths and load data and parameters
setup_paths

load('simoutGreyEstData.mat')

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


% LCoMPosxy = simout.greyEstCoMData.signals.values(:,1:2);
% RCoMPosxy = simout.greyEstCoMData.signals.values(:,3:4);
% HATCoMPosxy = simout.greyEstCoMData.signals.values(:,5:6);


% This constant is calculated in calculateSensorData.m
hipCorrection = 0.0816;
L = 1;
initialCWParameters
dt_visual = 1e-3;

%% Find heelstrike
GaitPhaseData       = simout.GaitPhaseData;
[LSwingIndices, RSwingIndices] = findSwingIndices(GaitPhaseData);

indices = cell2mat(RSwingIndices(swingNumber));

%% Sensor placement
% sensor_stance = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively
% sensor_hip = 'hip'; % Choose 'hip', 'hat', for the hip or the hat respectively
% sensor_swing = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively

if sensor_stance == 't'
    RfootPosxy = RToePosxy;
elseif sensor_stance == 'a'
    RfootPosxy = RAnklePosxy;
elseif sensor_stance == 'h'
    RfootPosxy = RHeelPosxy;
elseif sensor_stance == 'c'
    RfootPosxy = RCoMPosxy;
end

if sensor_swing == 't'
    LfootPosxy = LToePosxy;
elseif sensor_swing == 'a'
    LfootPosxy = LAnklePosxy;
elseif sensor_swing == 'h'
    LfootPosxy = LHeelPosxy;
elseif sensor_swing == 'c'
    LfootPosxy = LAnklePosxy;
end

% Calculate the hip correction, which is the displacement in x position of
% the hip to make the compass-gait biped hit the ground at the same time as
% the model.
RFootEnd = RfootPosxy(indices(end),1);
LFootEnd = LfootPosxy(indices(end),1);
RHipPosEnd = RHipPosxy(indices(end),1);
LHipPosEnd = LHipPosxy(indices(end),1);
hipPosEnd = (LHipPosEnd+RHipPosEnd)/2;
hipCorrection = hipPosEnd-(LFootEnd+RFootEnd)/2;

if strcmp(sensor_hip,'hip') 
    HipPosxy = (LHipPosxy + RHipPosxy)/2;
elseif strcmp(sensor_hip,'hipcor')      
    HipPosxy = (LHipPosxy + RHipPosxy)/2-hipCorrection;
elseif strcmp(sensor_hip,'hat') 
    HipPosxy = HATPosxy;
elseif strcmp(sensor_hip,'chat') 
    HipPosxy = HATCoMPosxy;
end

if strcmp(sensor_hip,'chat') && strcmp(sensor_stance,'c') && strcmp(sensor_swing,'c') 
    HipPosxy = (LHipPosxy + RHipPosxy)/2;
    th1 = atan((HipPosxy(:,1)-LCoMPosxy(:,1))./(HipPosxy(:,2)-LCoMPosxy(:,2)));
    th2 = atan((HipPosxy(:,1)-RCoMPosxy(:,1))./(HipPosxy(:,2)-RCoMPosxy(:,2)));
else
    th1 = asin((HipPosxy(:,1)-LfootPosxy(:,1))/L);
    th2 = asin((HipPosxy(:,1)-RfootPosxy(:,1))/L);
end

% Calculate dth1 and dth2
dth1 = gradient(th1,dt_visual);
dth2 = gradient(th2,dt_visual);

% Define x and u
x = [th1,th2,dth1,dth2];
u = cos(x(:,1:2));

% Cut data
tdata = t(indices);
LdataxNathan = x(indices,:);
Ldatau = u(indices,:);
Ndata = length(indices);
swingpercentagedataNathan = linspace(0,100,Ndata);

data = iddata(LdataxNathan, Ldatau, dt_visual');


set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

figure('DefaultAxesFontSize',11)
subplot(211); hold on
plot(swingpercentagedataNathan,LdataxNathan(:,1:2))
ylabel('Angle in rad')
legend('$\theta_1$', '$\theta_2$','location','east')

subplot(212)
plot(swingpercentagedataNathan,LdataxNathan(:,3:4))
ylabel('Angular velocity in rad/s')
xlabel('Swing in %')
legend('$\dot{\theta_1}$', '$\dot{\theta_2}$','location', 'southeast')

v = get(gca,'Position');
set(gca,'Position',[v(1) v(2)*1.5 v(3:4)])

%% Model song

% Choose data set
swingNumber = 15;
%% Setup paths and load data and parameters
setup_paths

loadGreyEstData
initialCWParameters
dt_visual = 1e-3;

%% Find heelstrike
GaitPhaseData       = simout.GaitPhaseData;
% [LSwingIndices, RSwingIndices] = findSwingIndices(GaitPhaseData);

RswingIdx = find(simout.RSwing.signals.values>0);
RswingEndIdx = find(diff(RswingIdx)>1);

RSwingIndices = cell(length(RswingEndIdx),1);
RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
for i = 1:length(RswingEndIdx)-1
    RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
end

indices = cell2mat(RSwingIndices(swingNumber));
endIndx = indices(end);

L = 1;

%% Sensor placement
% sensor_stance = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively
% sensor_hip = 'hip'; % Choose 'hip', 'hat', for the hip or the hat respectively
% sensor_swing = 'a'; % Choose 'h', 'a', 't' for hip ankle or toe respectively

figure('DefaultAxesFontSize',11); hold on

% Find hip correction
RfootPosxy = RAnklePosxy;
LfootPosxy = LAnklePosxy;
% Hip correction = hipPos - hipPosDesired
% Desired hipPos is in the middle of the two feet upon heelstrike
hipCorrection = (LHipPosxy(endIndx) + RHipPosxy(endIndx))/2-(RfootPosxy(endIndx)+LfootPosxy(endIndx))/2

HipPosxy = (LHipPosxy + RHipPosxy)/2 - hipCorrection; 

% Calculate state 
th1 = asin((HipPosxy(:,1)-LfootPosxy(:,1))/L);
th2 = asin((HipPosxy(:,1)-RfootPosxy(:,1))/L);

dth1 = gradient(th1,dt_visual);
dth2 = gradient(th2,dt_visual);

% Define x and u
x = [th1,th2,dth1,dth2];

% Cut data
tdata = t(indices);
LdataxSong = x(indices,:);
Ndata = length(indices);
swingpercentagedataSong = linspace(0,100,Ndata);

subplot(221)
hold on
plot(swingpercentagedataSong,LdataxSong(:,1),'LineWidth',1.5)
plot(swingpercentagedataNathan,LdataxNathan(:,1),'-.','LineWidth',1.5)
title('Stance leg')
ylabel('Angle in rad')
legend('Song','Nathan','Location', 'northwest')

subplot(222)
hold on
plot(swingpercentagedataSong,LdataxSong(:,2),'LineWidth',1.5)
plot(swingpercentagedataNathan,LdataxNathan(:,2),'-.','LineWidth',1.5)
title('Swing leg')

subplot(223)
hold on
plot(swingpercentagedataSong,LdataxSong(:,3),'LineWidth',1.5)
plot(swingpercentagedataNathan,LdataxNathan(:,3),'-.','LineWidth',1.5)

ylabel('Angular velocity in rad/s')
xlabel('Swing in %')

subplot(224)
hold on    
plot(swingpercentagedataSong,LdataxSong(:,4),'LineWidth',1.5)
plot(swingpercentagedataNathan,LdataxNathan(:,4),'-.','LineWidth',1.5)
ylabel('Angular velocity in rad/s')
xlabel('Swing in %')


