clear all
close all
clc

estimateSystem = 1;
%% Estimate the system
loadGreyEstData

if estimateSystem
    dataset = 'simoutGreyEstDataSong.mat'; % dataset used for identification
    swingNumber = 10;
%     skipSwingPercentage = 0;
%     virGravityDirs = 2;
    dt_visual = 1e-3;
%     [modelOpt, ParametersOpt] = CWgreyest(simout, dataset, skipSwingPercentage, ...
%                                             virGravityDirs, dt_visual);

    [modelOpt, ParametersOpt] = greyboxEstimation(dataset, swingNumber)
else
    load('Optimal_params_30percentSong.mat')
    dt_visual = 1e-3;
end
dt = 0.01;
Qy = 1e8;
yref = 0.7;
params = setupParamsStruct(ParametersOpt, Qy, yref, dt);

%% Validate model on other dataset
%%% Validation of prediction on the full dataset of the nonlinear model
datasetVal = 11;    
skipSwingPercentageVal = 0;

GaitPhaseData       = simout.GaitPhaseData;

RswingIdx = find(simout.RSwing.signals.values>0);
RswingEndIdx = find(diff(RswingIdx)>1);

RSwingIndices = cell(length(RswingEndIdx),1);
RSwingIndices{1} = RswingIdx(1):RswingIdx(RswingEndIdx(1));
for i = 1:length(RswingEndIdx)-1
    RSwingIndices{i+1} = RswingIdx(RswingEndIdx(i)+1):RswingIdx(RswingEndIdx(i+1));
end

% Find corresponding indices
allIndicesVal = cell2mat(RSwingIndices(datasetVal));
NdataVal = length(allIndicesVal);
skipIndicesVal = round(skipSwingPercentageVal/100*NdataVal);
cutIndicesVal = allIndicesVal(skipIndicesVal+1:end);
            
[allRdataxVal, allRdatauVal, allNdataVal] = calculateSensorData(LAnklePosxy,...
                RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,allIndicesVal);
[cutLdataxVal, cutLdatauVal, cutNdataVal] =calculateSensorData(LAnklePosxy,...
                RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,cutIndicesVal);


dataVal = iddata(allRdataxVal(:,1:2), allRdatauVal, dt_visual');

modelOpt.Initialstates(1).Value = allRdataxVal(1,1);
modelOpt.Initialstates(2).Value = allRdataxVal(1,2);
modelOpt.Initialstates(3).Value = allRdataxVal(1,3);
modelOpt.Initialstates(4).Value = allRdataxVal(1,4);
figure
compare(dataVal, modelOpt);

%% Validation of prediction over a finite horizon of the lin and nonlin model
horizon = 600;

x0 = allRdataxVal(1,:).';
swingPercentageData = linspace(0,100,allNdataVal);

                
m = params.m;
a = params.a;
mH = params.mH;
phi1 = params.phi1;
phi2 = params.phi2;


tspan = t(1:horizon+1);
[~,xnonlin] = ode45(@(t,x) CWodefun(t,x,m,a,mH,phi1,phi2), tspan,x0);

% figure
% plot(tspan,xnonlin)

%% Linearize model 1, linearization point
%%% Find linearization
xeq = x0;
Ac1 = Asymfun(xeq,0);
Bc1 = Bsymfun(xeq,0);
Baffc1 = Baffsymfun(xeq,0);

% Initial plot
xlin = x0;
for i = 1:horizon
    xdot(:,i) =Ac1*xlin(:,i)+Baffc1;
    xlin(:,i+1) = xlin(:,i)+xdot(:,i)*dt_visual;
end
cutSwingPercentageData = mapfun(1:1+horizon, 1, allNdataVal,0,100);


%% Linearize model 2, linearization point 50%
sw_cond = xnonlin(:,1)+xnonlin(:,2); % Switch condition th1+th2 = 0
crossIdx = find(sw_cond(2:end).*sw_cond(1:end-1)<0);

if isempty(crossIdx)
    crossIdx = 1;
end
crossIdx = crossIdx(end);

% The linearization index at alpha = 0.5
indx50 = round(crossIdx/2);
xeq = xnonlin(indx50,:).';

Ac2 = Asymfun(xeq,0);
Bc2 = Bsymfun(xeq,0);
Baffc2 = Baffsymfun(xeq,0);

% Initial plot
xlin2 = x0;
for i = 1:horizon
%     xdot(:,i) = CWodefun(0,xlin(:,i),m,a,mH,phi1,phi2);
    xdot2(:,i) =Ac2*xlin2(:,i)+Baffc2;
    xlin2(:,i+1) = xlin2(:,i)+xdot2(:,i)*dt_visual;
end
cutSwingPercentageData2 = mapfun(1:1+horizon, 1, allNdataVal,0,100);

N = length(allRdataxVal);
%% At halfway point of the swing
x0 = allRdataxVal(round(N/2),:).';

[~,xnonlin50] = ode45(@(t,x) CWodefun(t,x,m,a,mH,phi1,phi2), tspan,x0);

% Dynamics in the linearization point
Ac3 = Asymfun(xeq,0);
Bc3 = Bsymfun(xeq,0);
Baffc3 = Baffsymfun(xeq,0);

xlin3 = x0;
for i = 1:horizon
    xdot3(:,i) = Ac3*xlin3(:,i)+Baffc3;
    xlin3(:,i+1) = xlin3(:,i)+xdot3(:,i)*dt_visual;
end
cutSwingPercentageData = mapfun(round(N/2):round(N/2)+horizon, 1, allNdataVal,0,100);
Ncut = find(cutSwingPercentageData == 100);

% linearization point 2
sw_cond = xnonlin50(:,1)+xnonlin50(:,2); % Switch condition th1+th2 = 0
crossIdx = find(sw_cond(2:end).*sw_cond(1:end-1)<0);

if isempty(crossIdx)
    crossIdx = 1;
end
crossIdx = crossIdx(end);

% The linearization index at alpha = 0.5
xend = xnonlin(crossIdx,:).';
% indx50 = round(crossIdx/2);
alpha = 0.5;
xeq = x0*(1-alpha) + xend*alpha;

A4 = Asymfun(xeq,0);
B4 = Bsymfun(xeq,0);
Baff4 = Baffsymfun(xeq,0);

xlin4 = x0;
for i = 1:horizon
    xdot4(:,i) = A4*xlin4(:,i)+Baff4;
    xlin4(:,i+1) = xlin4(:,i)+xdot4(:,i)*dt_visual;
end

%% Plot for in thesis report
C = linspecer(5,'blue');
figure('DefaultAxesFontSize',11)
subplot(221)
hold on
plot(swingPercentageData,allRdataxVal(:,1),'g','LineWidth',1)
plot(swingPercentageData,xlin(1,1:N).',':','color',C(3,:),'LineWidth',2);
plot(swingPercentageData,xlin2(1,1:N).','-.','color',C(4,:),'LineWidth',1.7);
plot(swingPercentageData,xnonlin(1:N,1),'--','color',C(5,:),'LineWidth',1.3);
axis([0,100,-0.3,0.4])
ylabel('\theta_1 in rad')
title('Start at 0% swing')
legend('Data','Lin $\alpha$ = 0','Lin $\alpha$ = 0.5','Nonlin', 'defaultAxesTickLabelInterpreter','latex', 'location','northwest')

subplot(222)
hold on
plot(swingPercentageData,allRdataxVal(:,1),'g','LineWidth',1)
plot(cutSwingPercentageData(1:Ncut),xlin3(1,1:Ncut).',':','color',C(3,:),'LineWidth',2);
plot(cutSwingPercentageData(1:Ncut),xlin4(1,1:Ncut).','-.','color',C(4,:),'LineWidth',1.7);
plot(cutSwingPercentageData(1:Ncut),xnonlin50(1:Ncut,1),'--','color',C(5,:),'LineWidth',1.3);
axis([0,100,-0.3,0.4])
title('Start at 50% swing')

subplot(223)
hold on
plot(swingPercentageData,allRdataxVal(:,2),'g','LineWidth',1)
plot(swingPercentageData,xlin(2,1:N).',':','color',C(3,:),'LineWidth',2);
plot(swingPercentageData,xlin2(2,1:N).','-.','color',C(4,:),'LineWidth',1.7);
plot(swingPercentageData,xnonlin(1:N,2),'--','color',C(5,:),'LineWidth',1.3);
axis([0,100,-0.8,0.4])
ylabel('\theta_2 in rad')
xlabel('Swing in %')

subplot(224)
hold on 
plot(swingPercentageData,allRdataxVal(:,2),'g','LineWidth',1)
plot(cutSwingPercentageData(1:Ncut),xlin3(2,1:Ncut).',':','color',C(3,:),'LineWidth',2);
plot(cutSwingPercentageData(1:Ncut),xlin4(2,1:Ncut).','-.','color',C(4,:),'LineWidth',1.7);
plot(cutSwingPercentageData(1:Ncut),xnonlin50(1:Ncut,2),'--','color',C(5,:),'LineWidth',1.3);
axis([0,100,-0.8,0.4])
xlabel('Swing in %')

%% Table for thesis report
cost_func = 'NRMSE';
names = {'lin 1';'lin 2'; 'nonlin'};
subplot1 = zeros(3,1);
subplot2 = zeros(3,1);
subplot3 = zeros(3,1);
subplot4 = zeros(3,1);

% Subplot 1
ydes = allRdataxVal(:,1);
y = xlin(1,1:N).';
subplot1(1) = goodnessOfFit(y,ydes,cost_func);
y = xlin2(1,1:N).';
subplot1(2) = goodnessOfFit(y,ydes,cost_func); 
y = xnonlin(1:N,1);
subplot1(3) = goodnessOfFit(y,ydes,cost_func); 

% Subplot 2
ydes = allRdataxVal(round(N/2):end,1);
y = xlin3(1,1:Ncut).';
subplot2(1) = goodnessOfFit(y,ydes,cost_func);
y = xlin4(1,1:Ncut).';
subplot2(2) = goodnessOfFit(y,ydes,cost_func); 
y = xnonlin50(1:Ncut,1);
subplot2(3) = goodnessOfFit(y,ydes,cost_func); 

% Subplot 3
ydes = allRdataxVal(:,2);
y = xlin(2,1:N).';
subplot3(1) = goodnessOfFit(y,ydes,cost_func);
y = xlin2(2,1:N).';
subplot3(2) = goodnessOfFit(y,ydes,cost_func); 
y = xnonlin(1:N,2);
subplot3(3) = goodnessOfFit(y,ydes,cost_func); 

% Subplot 4
ydes = allRdataxVal(round(N/2):end,2);
y = xlin3(2,1:Ncut).';
subplot4(1) = goodnessOfFit(y,ydes,cost_func);
y = xlin4(2,1:Ncut).';
subplot4(2) = goodnessOfFit(y,ydes,cost_func); 
y = xnonlin50(1:Ncut,2);
subplot4(3) = goodnessOfFit(y,ydes,cost_func); 

T = table(names,subplot1,subplot2,subplot3,subplot4)

% subplot(222)
% hold on
% plot(swingPercentageData,allRdataxVal(:,1),'g','LineWidth',1.5)
% plot(cutSwingPercentageData(1:Ncut),xlin3(1,1:Ncut).','color',C(3,:),'LineWidth',1.7);
% plot(cutSwingPercentageData(1:Ncut),xlin4(1,1:Ncut).','-.','color',C(4,:),'LineWidth',1.7);
% plot(cutSwingPercentageData(1:Ncut),xnonlin50(1:Ncut,1),'--','color',C(5,:),'LineWidth',1.3);
% axis([0,100,-0.3,0.4])
% title('Start at 50% swing')

%% Computation time thesis report
%NMPC
xstart = allRdataxVal(1,:);
kimpStart = 50;
kimpMid = 25;
xmid = allRdataxVal(round(N/2),:);

f = @() NMPC(xstart.', kimpStart, params);
timeit(f)
% tElapsed = zeros(100,1);
% for i = 1:100
%     tic
%     ystart = NMPC(xstart.', kimpStart, params);
%     tElapsed(i) = toc;
% end
% mean(tElapsed)
% 
% tElapsed = zeros(100,1);
% for i = 1:100
%     tic
%     ystart = NMPC(xmid.', kimpMid, params);
%     tElapsed(i) = toc;
% end
% mean(tElapsed)
%% Compare linear model with nonlinear model in an interactive plot  
%%% PLOT FIGURE
% Set interpreter for plot to LaTeX
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

% figure options
f = figure;
ax = axes('Parent',f,'position',[0.13 0.29  0.77 0.64]);
xlabel('Swing in %')
ylabel('Angle in rad')
slider = uicontrol('Parent',f,'Style','slider','Position',[100,50,620,23],...
              'value',1, 'min',1, 'max',allNdataVal);
slider.UserData = cutLdataxVal;
slider.Callback = @plotLines;
set(gcf,'position',[50,50,800,600]);

% plot data
hold on
plot(swingPercentageData,allRdataxVal(:,1:2),'g','LineWidth',1.5)
nonlinPrediction = plot(cutSwingPercentageData,xnonlin(:,1:2),'k','LineWidth',1.3);
linPrediction = plot(cutSwingPercentageData,xlin(1:2,:).','b--','LineWidth',1.7);
linPrediction2 = plot(cutSwingPercentageData,xlin2(1:2,:).','r--','LineWidth',1.7);
legend('$\theta_1$','$\theta_2$','$\theta_1$ nonlin','$\theta_2$ nonlin', ...
    '$\theta_1$ lin $\alpha = 0$','$\theta_2$ lin $\alpha = 0$', ...
    '$\theta_1$ lin $\alpha = 0.5$','$\theta_2$ lin $\alpha = 0.5$','location','north')
axis([0,100,-0.8,0.5])

% save data to use in the plotLines callback function
setappdata(f,'horizon',horizon);
setappdata(f,'allRdatax',allRdataxVal);
setappdata(f,'dt_visual',dt_visual);
setappdata(f,'allNdata',allNdataVal);
setappdata(f,'Asym',Asym);
setappdata(f,'Baffsym',Baffsym);
setappdata(f,'linPrediction',linPrediction);
setappdata(f,'linPrediction2',linPrediction2);
setappdata(f,'f',f);
setappdata(f,'linPrediction',linPrediction);
setappdata(f,'nonlinPrediction',nonlinPrediction);
setappdata(f,'ParametersOpt',ParametersOpt);
setappdata(f,'t',t);
%%
eval(ddth_nonlin)
Ac1*(x0-x0)+Baffc1
dx = CWodefun(0,x0,m,a,mH,phi1,phi2)

xlin(:,2)
xnonlin(:,2)

%%
function plotLines(src,event)
    % This is the callback function that is called when the slider is moved
    k = round(src.Value); % This is the value of the slider
    
    % Extract all required parameters and data
	horizon = getappdata(src.Parent,'horizon');
	allRdatax = getappdata(src.Parent,'allRdatax');
	dt_visual = getappdata(src.Parent,'dt_visual');
	allNdata = getappdata(src.Parent,'allNdata');
	Asym = getappdata(src.Parent,'Asym');
	Baffsym = getappdata(src.Parent,'Baffsym');
	linPrediction = getappdata(src.Parent,'linPrediction');
	linPrediction2 = getappdata(src.Parent,'linPrediction2');
	nonlinPrediction = getappdata(src.Parent,'nonlinPrediction');
	ParametersOpt = getappdata(src.Parent,'ParametersOpt');
	t = getappdata(src.Parent,'t');
	f = getappdata(src.Parent,'f');
    
    N = length(allRdatax);
   
    tspan = t(k:k+horizon);
    
    x0 = allRdatax(k,:).';
        
    % Nonlinear prediction
    m = ParametersOpt(1);
    a =  ParametersOpt(2);
    mH =  ParametersOpt(3);
    phi1 =  ParametersOpt(4);
    if length(ParametersOpt)>4
        phi2 =  ParametersOpt(5);
    else
        phi2 = phi1;
    end
    
    [~,xnonlin] = ode45(@(t,x) CWodefun(t,x,m,a,mH,phi1,phi2), tspan,x0);
    
    % linearization point 1
    th1 = allRdatax(k,1);
    th2 = allRdatax(k,2);
    dth1 = allRdatax(k,3);
    dth2 = allRdatax(k,4);
    
    uCMG = 0;
    CMGangle = 0;

    % Dynamics in the linearization point
    A1 = eval(Asym);
    Baff1 = eval(Baffsym);
    
    xlin = x0;
    for i = 1:horizon
        xdot(:,i) = A1*xlin(:,i)+Baff1;
        xlin(:,i+1) = xlin(:,i)+xdot(:,i)*dt_visual;
    end
    cutSwingPercentageData = mapfun(k:k+horizon, 1, allNdata,0,100);
    
   
    % linearization point 2
    th1 = allRdatax(round((k+N)/2),1);
    th2 = allRdatax(round((k+N)/2),2);
    dth1 = allRdatax(round((k+N)/2),3);
    dth2 = allRdatax(round((k+N)/2),4);
    
    A2 = eval(Asym);
    Baff2 = eval(Baffsym); 
    
    xlin2 = x0;
    for i = 1:horizon
        xdot2(:,i) = A2*xlin2(:,i)+Baff2;
        xlin2(:,i+1) = xlin2(:,i)+xdot2(:,i)*dt_visual;
    end
    
    y = allRdatax(k:end,1);
    yhat = xnonlin(1:N-k+1,1);
    vaf =(1-sum((y-yhat).^2)/(sum(y.^2)))* 100
    vaf2 =(1-(var(y-yhat)/var(y)) )* 100
    
    delete(linPrediction)
    delete(linPrediction2)
    delete(nonlinPrediction)
    nonlinPrediction = plot(cutSwingPercentageData,xnonlin(:,1:2),'k','LineWidth',1.3);
    linPrediction = plot(cutSwingPercentageData,xlin(1:2,:).','b--','LineWidth',1.7);
    linPrediction2 = plot(cutSwingPercentageData,xlin2(1:2,:).','r--','LineWidth',1.7);
legend('$\theta_1$','$\theta_2$','$\theta_1$ nonlin','$\theta_2$ nonlin', ...
    '$\theta_1$ lin $\alpha = 0$','$\theta_2$ lin $\alpha = 0$', ...
    '$\theta_1$ lin $\alpha = 0.5$','$\theta_2$ lin $\alpha = 0.5$','location','north')
    axis([0,100,-0.7,0.5])
    setappdata(f,'linPrediction',linPrediction);
    setappdata(f,'linPrediction2',linPrediction2);
    setappdata(f,'nonlinPrediction',nonlinPrediction);
    
end