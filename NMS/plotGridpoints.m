clear all
close all
clc

load('200gridpoints.mat')

figure
hold on
plot(ugrid,Jgrid,'k','linewidth',2)
xlabel('{\it u}_c_o_n in Nm')
ylabel('{\it J}_2 in m^2')