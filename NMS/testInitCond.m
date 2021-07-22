clear all
close all
clc

load('nocontrolLong3.mat')  
yref = 0.8;

%% Run from normal initial condition
% Settings from file:
% % initial locomotion speed
% vx0 = 1.3; %[m/s] 
% 
% % left (stance) leg ankle, knee and hip joint angles
% Lphi120  =  85*pi/180; %[rad]
% Lphi230  = 175*pi/180; %[rad]
% Lphi340  = 175*pi/180; %[rad]
% 
% % right (swing) leg ankle, knee and hip joint angles
% Rphi120  =  90*pi/180; %[rad]
% Rphi230  = 175*pi/180; %[rad]
% Rphi340  = 140*pi/180; %[rad]

swingIndx0 = 1;
% The initial conditions needs 
% * Left foot pos & vel
LFootPos = simout.LFootPos.signals.values;
LFootAng = simout.LFootAng.signals.values;
x0 = LFootPos(swingIndx0,1);
vx0 = LFootPos(swingIndx0,4);
y0 = LFootPos(swingIndx0,2);
dy0 = LFootPos(swingIndx0,5);
phi0 = LFootAng(swingIndx0,1);
dphi0 = LFootAng(swingIndx0,2);


% * Left ankle ang & vel
AngLAnk = simout.AngLAnk.signals.values;
Lphi120  =  AngLAnk(swingIndx0,1); %[rad]
Ldphi120  =  AngLAnk(swingIndx0,2); %[rad/s]

% * Left knee ang & vel
AngLKnee = simout.AngLKnee.signals.values;
Lphi230  =  AngLKnee(swingIndx0,1); %[rad]
Ldphi230  =  AngLKnee(swingIndx0,2); %[rad/s]
% * Left hip ang & vel
AngLHip = simout.AngLHip.signals.values;
Lphi340  =  AngLHip(swingIndx0,1); %[rad]
Ldphi340  =  AngLHip(swingIndx0,2); %[rad/s]
% * Right ankle ang & vel
AngRAnk = simout.AngRAnk.signals.values;
Rphi120  =  AngRAnk(swingIndx0,1); %[rad]
Rdphi120  =  AngRAnk(swingIndx0,2); %[rad/s]
% * Right knee ang & vel
AngRKnee = simout.AngRKnee.signals.values;
Rphi230  =  AngRKnee(swingIndx0,1); %[rad]
Rdphi230  =  AngRKnee(swingIndx0,2); %[rad/s]
% * Right hip ang & vel
AngRHip = simout.AngRHip.signals.values;
Rphi340  =  AngRHip(swingIndx0,1); %[rad]
Rdphi340  =  AngRHip(swingIndx0,2); %[rad/s]

nms_model_MechInit;
nms_model_ControlInit;

dt_visual = 0.01;
model = 'nms_modelv2';
simout = sim(model,'SrcWorkspace','current');

%% Find last swing data
RFootContact = simout.RFootContact.signals.values;
Indx = find(diff(RFootContact>0));
for i = length(Indx)-2:-1:1
    if any(RFootContact(Indx(i)+1:(Indx(i+1))) > 0)
        % swingIndx contains all indices of the last swing
        swingIndx = Indx(i)+1:(Indx(i+1));
        break
    else
        continue
    end
end

% Find the indices of the third swing
indices3 = Indx(4):Indx(5);

stepLength =  simout.RFootPos.signals.values(indices3(end),1)-simout.LFootPos.signals.values(indices3(end),1)
Jnow = (stepLength-yref)^2
J(i) = Jnow;

t = 0:0.01:6;
t = t';

%% Run from initial condition last swing

% Settings from file:
% % initial locomotion speed
% vx0 = 1.3; %[m/s] 
% 
% % left (stance) leg ankle, knee and hip joint angles
% Lphi120  =  85*pi/180; %[rad]
% Lphi230  = 175*pi/180; %[rad]
% Lphi340  = 175*pi/180; %[rad]
% 
% % right (swing) leg ankle, knee and hip joint angles
% Rphi120  =  90*pi/180; %[rad]
% Rphi230  = 175*pi/180; %[rad]
% Rphi340  = 140*pi/180; %[rad]
swingIndx0new = indices3(1);
% The initial conditions needs 
% * Left foot pos & vel
LFootPos = simout.LFootPos.signals.values;
LFootAng = simout.LFootAng.signals.values;
x0 = LFootPos(swingIndx0new,1);
vx0 = LFootPos(swingIndx0new,4);
y0 = LFootPos(swingIndx0new,2);
dy0 = LFootPos(swingIndx0new,5);
phi0 = LFootAng(swingIndx0new,1);
dphi0 = LFootAng(swingIndx0new,2);


% * Left ankle ang & vel
AngLAnk = simout.AngLAnk.signals.values;
Lphi120  =  AngLAnk(swingIndx0new,1); %[rad]
Ldphi120  =  AngLAnk(swingIndx0new,2); %[rad/s]

% * Left knee ang & vel
AngLKnee = simout.AngLKnee.signals.values;
Lphi230  =  AngLKnee(swingIndx0new,1); %[rad]
Ldphi230  =  AngLKnee(swingIndx0new,2); %[rad/s]
% * Left hip ang & vel
AngLHip = simout.AngLHip.signals.values;
Lphi340  =  AngLHip(swingIndx0new,1); %[rad]
Ldphi340  =  AngLHip(swingIndx0new,2); %[rad/s]
% * Right ankle ang & vel
AngRAnk = simout.AngRAnk.signals.values;
Rphi120  =  AngRAnk(swingIndx0new,1); %[rad]
Rdphi120  =  AngRAnk(swingIndx0new,2); %[rad/s]
% * Right knee ang & vel
AngRKnee = simout.AngRKnee.signals.values;
Rphi230  =  AngRKnee(swingIndx0new,1); %[rad]
Rdphi230  =  AngRKnee(swingIndx0new,2); %[rad/s]
% * Right hip ang & vel
AngRHip = simout.AngRHip.signals.values;
Rphi340  =  AngRHip(swingIndx0new,1); %[rad]
Rdphi340  =  AngRHip(swingIndx0new,2); %[rad/s]

%%
nms_model_MechInit;
nms_model_ControlInit;

dt_visual = 0.01;
model = 'nms_modelv2';
simoutnew = sim(model,'SrcWorkspace','current');

%% Check if runs overlap
% the angles should be the same
AngLAnk = simout.AngLAnk.signals.values;
AngLAnknew = simoutnew.AngLAnk.signals.values;
Lphi120  =  AngLAnk(swingIndx0new,1) %[rad]
Lphi120  =  AngLAnknew(1,1) %[rad]

% Check stimulation signals
simout.stimSignals.Data(swingIndx0new,:)
simoutnew.stimSignals.Data(1,:)
