clear all
close all
clc

nms_model_MechInit;
nms_model_ControlInit;
initState;

dt_visual = 0.01;

model = 'nms_modelv2';

uConstant = 0;

% * Left ankle ang & vel
x0 = 0; % [m]
y0 = 0;
dy0 = 0;
phi0 = 0;
dphi0 = 0;
Ldphi120  =  0; %[rad/s]
Ldphi230  =  0; %[rad/s]
Ldphi340  =  0; %[rad/s]
Rdphi120  =  0; %[rad/s]
Rdphi230  =  0; %[rad/s]
Rdphi340  =  0; %[rad/s]

simout = sim(model,'SrcWorkspace','current');