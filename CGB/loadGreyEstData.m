% This file loads all relevant data for grey box estimation, but also for
% the sensor placement
load('simoutGreyEstDataSong.mat')
% load('nocontrol3.mat')


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