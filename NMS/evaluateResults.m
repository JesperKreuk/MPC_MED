clear all
close all
clc

%% Load data
load('largeOptimization6.mat')

%% Calculate uncertainty w
% Find the minimum of the found grid by fitting a quadratic function

delta = 1.0335e-06;
for i = 1:length(allJ)
    if isempty(allugrid{i})
        w(i) = 0;
    else
        ugrid = cell2mat(allugrid(i));
        Jgrid = cell2mat(allJgrid(i));
        p = polyfit(ugrid,Jgrid,2);
        w(i) = sqrt((allJ{i}+delta)/p(1));
    end
end
%% Plot figure
figure
hold on

xdisturbance = -50:5:50;
xdisturbance(11) = [];
for i = 1:7
    errorbar(xdisturbance,cell2mat(alluopt(20*(i-1)+1:20*i)),w(20*(i-1)+1:20*i))
end

xlabel('Disturbance amplitude in Nm') 
ylabel('uopt in Nm')
title('Constant input from toe-off 3th swing') 
legend('Foot stance', 'Shank stance', 'Thigh stance', 'HAT', 'Thigh swing', 'Shank swing', 'HAT push')

%% changing reference
clear all
clc

load('changingRefOpt.mat')
delta = 1.0335e-06;
for i = 1:length(allJ)
    if isempty(allugrid{i})
        w(i) = 0;
    else
        ugrid = cell2mat(allugrid(i));
        Jgrid = cell2mat(allJgrid(i));
        p = polyfit(ugrid,Jgrid,2);
        w(i) = sqrt((allJ{i}+delta)/p(1));
    end
end

figure
hold on

%%
ugrid = cell2mat(allugrid(10));
Jgrid = cell2mat(allJgrid(10));
figure
plot(ugrid,Jgrid)
xlabel('u in Nm')
ylabel('J in m^2')
%%
for i = 1:7
    errorbar(references,cell2mat(alluopt),w)
end

xlabel('reference in m')
ylabel('uopt in Nm') 
title('Constant input from toe-off 3th swing') 















