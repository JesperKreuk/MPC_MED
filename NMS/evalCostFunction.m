clear all
close all
clc
%% Parameters
nms_model_MechInit;
nms_model_ControlInit;
initState;
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
dt_visual = 0.01;
%% Optimization settings
yref = 0.69;

model = 'nms_model_modified';

%% Simulation of optimal result
u = 0:0.02:3;
for i = 1:length(u)
    uConstant = 0; %u(i)

    tic
    simout = sim(model,'SrcWorkspace','current');
    toc

    stepLength =  simout.RFootPos.signals.values(end,1)-simout.LFootPos.signals.values(end,1)
    Jnow = (stepLength-yref)^2
    J(i) = Jnow;
end

%% Plot figure
figure
plot(u,J)
xlabel('input u')
ylabel('J')
title('Cost function J = (y-yref)^2')

%% Simulate spikes
uConstant = u(143)

tic
simout = sim(model,'SrcWorkspace','current');
toc