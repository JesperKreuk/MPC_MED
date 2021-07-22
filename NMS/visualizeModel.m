clear all
close all
clc

%% Parameters
nms_model_MechInit;
nms_model_ControlInit;

dt_visual = 0.01;
uConstant = 0;
%% Run model
model = 'nms_model_modified';
simout = sim(model);



%% Joint positions
load('nocontrol3.mat')

figure
plot(AngRAnk.signals.values(:,1)./pi*180)

LFootPosxy = LFootPos.signals.values(:,1:2);

LHipPosxy = xyzLHip.signals.values(:,1:2);
RHipPosxy = xyzRHip.signals.values(:,1:2);

HipPosxy = (LHipPosxy + RHipPosxy)./2;

RFootPosxy = RFootPos.signals.values(:,1:2);

N = length(LHipPosxy);
i = 1;
walker = plot([LFootPosxy(i,1),HipPosxy(i,1),RFootPosxy(i,1)],[LFootPosxy(i,2),HipPosxy(i,2),RFootPosxy(i,2)],'k-o');
    
figure
for i = 2:N 
    delete(walker)
    walker = plot([LFootPosxy(i,1),HipPosxy(i,1),RFootPosxy(i,1)],[LFootPosxy(i,2),HipPosxy(i,2),RFootPosxy(i,2)],'k-o');
    axis equal
    ylim([0 1])
    drawnow
end


