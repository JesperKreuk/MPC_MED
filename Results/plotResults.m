clear all
close all
clc

%% Load results
load('CGBmld0results.mat')
load('CGBmld50results.mat')
load('CGBnonlinresults.mat')

%% Plot results CGB controllers
% Load colors from a parameter file
Colors;

figure('DefaultAxesFontSize',11)
hold on
h = zeros(1,5);
h(1) = plot([0,100],[0,0],'color',Color2(3,:),'LineWidth',1);
h(2) = plot([0,100],[5,5],'color',Color2(3,:),'LineWidth',1);
h(3) = plot(swingPercent0mld,torque0mld,':','LineWidth', 2.5,'color',Color1(3,:));
h(4) = plot(swingPercent50mld,torque50mld,'-.','LineWidth',2.5,'color',Color1(4,:));
h(5) = plot(swingPercentNonlin,torqueNonlin,'--','LineWidth',2.5,'color',Color1(5,:));
legend([h(3:5),h(2)], 'Lin \alpha = 0','Lin \alpha = 0.5','Nonlin','Bounds', 'location','southeast')


xlabel('Swing in %')
ylabel('Torque in Nm')
axis([0,100,-1,6])
