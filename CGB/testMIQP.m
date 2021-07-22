clear all
close all
clc

% This script will test the MIQP that was build 
% The following 2 functions should be equal:
% * The cost function Jsim = (y(1)-yref)Qy(y(1)-yref)
%       y(1) is from the simulation
% * The cost function Jgur found by the Gurobi optimization 

% Test 1 
% * xP = 1
% * N = 1
% * yref = 1 % This is not reachable
% * Qy = 1
% * xeq = 0 

% Test 2 
% * xP = 1
% * N = 10
% * yref = 0.01 This is reachable from xeq
% * Qy, Qu can be chosen freely, Qy >> Qu should get yend = yref
% * xeq = 0 

% Test 3 
% * xP = 1
% * N = 10
% * yref =  2*pi/10+0.01
% * Qy = 1
% * xeq = [pi/10, -pi/8, 0, 0, 0] (th_4x1,gamma) 


%% Load model
% Load the general model
% load('ModelLinSym.mat', 'Asym', 'Bsym', 'ddth_nonlin')
dt = 1e-2;
L = 1;

%% Model 1: x = 0 (Test 1 and 2)
xeq1 = zeros(4,1);
Abarc1 = Asymfun(xeq1,0);
Bubarc1 = Bsymfun(xeq1,0);
Baffbarc1 =  Baffsymfun(xeq1,0);
% [Abar1, Bubar1, Baffbar1] = subsLinModel(xeq1, dt);
S1 = buildMLD(Abarc1, Bubarc1, Baffbarc1, dt);

%% Model 2: x = [pi/10; -pi/8; 0; -0.7] (Test 3)
xeq2 = [pi/10; -pi/8; 0; -0.7];

Abarc2 = Asymfun(xeq2,0);
Bubarc2 = Bsymfun(xeq2,0);
Baffbarc2 =  Baffsymfun(xeq2,0);
S2 = buildMLD(Abarc2, Bubarc2, Baffbarc2, dt);

%% Model 3: x from actual data (Test 4)
% Load actual data
loadGreyEstData;

dataset = 10;    

% Load general swing data
GaitPhaseData       = simout.GaitPhaseData;

RswingIdx = find(simout.RSwing.signals.values>0);
RswingEndIdx = find(diff(RswingIdx)>1);

RSwingIndices = cell(length(RswingEndIdx),1);
RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
for i = 1:length(RswingEndIdx)-1
    RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
end

% Only look at the end of the swing
allIndices = cell2mat(RSwingIndices(dataset));
% last50Indices = allIndices(end-49:end);

% Calculate angles theta at the end of the swing
[Rdatax, Rdatau, Ndata] = calculateSensorData(LAnklePosxy,...
                RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,allIndices);
xenddata =  [Rdatax(end,:).';0;1];

xeq3 = Rdatax(250,:).'; 
Abarc3 = Asymfun(xeq3,0);
Bubarc3 = Bsymfun(xeq3,0);
Baffbarc3 =  Baffsymfun(xeq3,0);
S3 = buildMLD(Abarc3, Bubarc3, Baffbarc3, dt);

%% Model 4: x from actual data at begin of the swing
xeq4 = Rdatax(1,:).'; 
Abarc4 = Asymfun(xeq3,0);
Bubarc4 = Bsymfun(xeq3,0);
Baffbarc4 =  Baffsymfun(xeq3,0);
S4 = buildMLD(Abarc4, Bubarc4, Baffbarc4, dt);

%% Test 1
% Cost function
Qy = 1;
Qu = 1;
yref = 1; 
N = 1; % Horizon

x0 = [xeq1;1]; % xP = 1

[H, f, constant, Acon, bcon, ivar, lb, ub] = buildMIQP(S1, Qy, Qu, x0, yref, N);

% Setup Gurobi
model = setupGurobi(H, f, constant, Acon, bcon, ivar, lb, ub);
params.outputflag = 0;

% Optimize V = [u;w] with Gurobi
result = gurobi(model, params);
uopt = result.x(1:N);

% Simulate the system with uopt
[xend,y,flag] = multiplestepMLD(S1,x0,uopt,N);
yend = y(end);

if flag
    disp('Flag error, some inequality was not satisfied in the MLD steps')
end

Jgur = result.objval;
Jsim = (yend-yref)'*Qy*(yend-yref)+uopt'*Qu*uopt;

disp('Test 1: optimize input for 1 time step')
if isequal(Jgur,Jsim) 
    disp('... successful! Jgur = Jsim')
    disp(result.status)
else
    disp('... Error! Jgur =/= Jsim')
%     disp(result.status)
end


%% Test 2
% Model 1: x = 0
% Cost function
Qy = 1e8;
yref = 0.01; 
N = 3; % Horizon
Qu = 1*eye(N); 

x0 = [xeq1;1]; % xP = 1

[H, f, constant, Acon, bcon, ivar, lb, ub] = buildMIQP(S1, Qy, Qu, x0, yref, N);

% Setup Gurobi
model = setupGurobi(H, f, constant, Acon, bcon, ivar, lb, ub);
params.outputflag = 0;

% Optimize V = [u;w] with Gurobi
result = gurobi(model, params);
uopt = result.x(1:N);

% Simulate the system with uopt
[xend,y,flag] = multiplestepMLD(S1,x0,uopt,N);
yend = y(end);

if flag
    disp('Flag error, some inequality was not satisfied in the MLD steps')
end

Jgur = result.objval;
Jsim = (yend-yref)'*Qy*(yend-yref)+uopt'*Qu*uopt;

disp('Test 2: optimize input for 10 time steps')
if isequal(Jgur,Jsim) 
    disp('... successful! Jgur = Jsim')
    disp(result.status)
elseif norm(Jgur-Jsim) < 1e-6    
    disp('... successful! Jgur - Jsim < 1e-6')
    disp(result.status)
else
    disp('... Error! Jgur =/= Jsim')
%     disp(result.status)
end

%% Test 3
% Model 2
x0 = [xeq2;1]; % xP = 1
y0 = S2.C*x0;

% Cost function
Qy = 1e8;
yref = S2.C*x0+0.01; 
N = 10; % Horizon
Qu = eye(N); 

[H, f, constant, Acon, bcon, ivar, lb, ub] = buildMIQP(S2, Qy, Qu, x0, yref, N);

% Setup Gurobi
model = setupGurobi(H, f, constant, Acon, bcon, ivar, lb, ub);
params.outputflag = 0;
% params.FeasibilityTol = 1e-4;

% Optimize V = [u;w] with Gurobi
result = gurobi(model, params);
% iis = gurobi_iis(model, params)
uopt = result.x(1:N);
% Simulate the system with uopt
[x, y, flag] = multiplestepMLD(S2,x0,uopt,N);
yend = y(end);

% if flag
%     disp('Flag error, some inequality was not satisfied in the MLD steps')
% end
    
Jgur = result.objval;
Jsim = (yend-yref)'*Qy*(yend-yref)+uopt'*Qu*uopt;

disp('Test 3: optimize input for 10 time steps from x=/=0')
if isequal(Jgur,Jsim) 
    disp('... successful! Jgur = Jsim')
    disp(result.status)
elseif norm(Jgur-Jsim) < 1e-6    
    disp('... successful! Jgur - Jsim < 1e-6')
    disp(result.status)
    (Jgur-Jsim)/Jgur
else
    disp('... Error! Jgur =/= Jsim')
%     disp(result.status)
end
% %% error analysis
% wgur = result.x(N+1:end)
% naux = size(S2.Baux,2)
% wgur = reshape(wgur,[naux,length(wgur)/naux])
% 
% wsim = []
% for i = 1:N
%    wsim(:,i) = x(1:5,i)*x(6,i) 
% end

%% Test 4
% Actual data just before impact
% Model 3
x0 = [xeq3;1]; % xP = 1
y0 = S3.C*x0;

% Cost function
Qy = 1e8;
yref =0.8; 
N = 20; % Horizon
Qu = eye(N); 


[H, f, constant, Acon, bcon, ivar, lb, ub] = buildMIQP(S3, Qy, Qu, x0, yref, N);

% Setup Gurobi
model = setupGurobi(H, f, constant, Acon, bcon, ivar, lb, ub);
params.outputflag = 0;

% Optimize V = [u;w] with Gurobi
result = gurobi(model, params);
uopt = result.x(1:N);

% Simulate the system with uopt
[xend,y,flag] = multiplestepMLD(S3,x0,uopt,N);
yend = y(end);

% if flag
%     disp('Flag error, some inequality was not satisfied in the MLD steps')
% end

% %% auxiliary check
% wgur = result.x(N+1:end);
% wgur = reshape(wgur,[9,20]);
% 
% wsim-wgur;

Jgur = result.objval;
Jsim = (y(end)-yref)'*Qy*(y(end)-yref)+uopt'*Qu*uopt;

disp('Test 4: optimize input for 20 time steps from x=/=0 with switching')
if isequal(Jgur,Jsim) 
    disp('... successful! Jgur = Jsim')
    disp(result.status)
elseif norm(Jgur-Jsim) < 1e-6    
    disp('... successful! Jgur - Jsim < 1e-6')
    disp(result.status)
    (Jgur-Jsim)/Jgur
else
    disp('... Error! Jgur =/= Jsim')
%     disp(result.status)
end
%% Simulation
% figure
% LFootVisualx = zeros(N+1,1);
% LFootVisualy = zeros(N+1,1);  %RfootPosxy(indices,2);
% HipVisualx =LFootVisualx + L*sin(x(1,:).');
% HipVisualy = LFootVisualy + L*cos(x(1,:).');
% RFootVisualx = HipVisualx-L*sin(x(2,:).');
% RFootVisualy = HipVisualy-L*cos(x(2,:).');
% 
% i = 1;
% walker = plot([LFootVisualx(i),HipVisualx(i),RFootVisualx(i)],[LFootVisualy(i),HipVisualy(i),RFootVisualy(i)],'o-','Color','r');
% 
% axis equal
% drawnow
% 
% for i= 2:length(HipVisualx)
%     delete(walker)
%     walker = plot([LFootVisualx(i),HipVisualx(i),RFootVisualx(i)],[LFootVisualy(i),HipVisualy(i),RFootVisualy(i)],'o-','Color','r');
%     axis equal
%     drawnow
%     pause(0.5)
% end

%% Model 4 test 
x0 = [xeq4;1]; % xP = 1
y0 = S4.C*x0;

% Cost function
Qy = 1e8;
yref =0.8; 
N = 50; % Horizon
Qu = eye(N); 


[H, f, constant, Acon, bcon, ivar, lb, ub] = buildMIQP(S4, Qy, Qu, x0, yref, N);

% Setup Gurobi
model = setupGurobi(H, f, constant, Acon, bcon, ivar, lb, ub);
params.outputflag = 0;

% Optimize V = [u;w] with Gurobi
tic
result = gurobi(model, params);
toc
uopt = result.x(1:N);

% Simulate the system with uopt
[xend,y,flag] = multiplestepMLD(S4,x0,uopt,N);
yend = y(end);