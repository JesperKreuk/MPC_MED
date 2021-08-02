% This is the callback function that is called when the slider is moved
% in the interactive plot greyboxCGB.m. It updates the plot depending
% on what initial condition is chosen.

% Author: Jesper Kreuk

function plotLines(src,event)
    k = round(src.Value); % This is the value of the slider
    
    % Extract all required parameters and data
	horizon = getappdata(src.Parent,'horizon');
	Rdatax = getappdata(src.Parent,'Rdatax');
	dt = getappdata(src.Parent,'dt');
	Ndata = getappdata(src.Parent,'Ndata');
	Nmodel = getappdata(src.Parent,'Nmodel');
	linPrediction = getappdata(src.Parent,'linPrediction');
	linPrediction2 = getappdata(src.Parent,'linPrediction2');
	nonlinPrediction = getappdata(src.Parent,'nonlinPrediction');
	ParametersOpt = getappdata(src.Parent,'ParametersOpt');
	f = getappdata(src.Parent,'f');
    
    % Initial state
    x0 = Rdatax(k,:).';
        
    % Parameters of the CGB model
    m = ParametersOpt(1);
    a =  ParametersOpt(2);
    mH =  ParametersOpt(3);
    phi1 =  ParametersOpt(4);
    phi2 =  ParametersOpt(5);
    
    % Simulate nonlinear CGB model
    tspan = 0:dt:0.6;
    N = length(tspan);
    [~,xnonlin] = ode45(@(t,x)dynamics(t,x,m,a,mH,phi1,phi2), tspan, x0);
    xnonlin = xnonlin.';
    
    % Normalise time vector
    cutSwingPercentageData = mapfun(k:10:k+N*10-1, 1, Ndata,0,100);
    
    % Linearise with lambda = 0
    xeq1 = x0;
    A1 = Asymfun(xeq1,0);
    Baff1 = Baffsymfun(xeq1,0);
    
    % Simulation of linear model with lambda = 0
    xlin1 = x0;
    for i = 1:horizon
        xdot1(:,i) = A1*xlin1(:,i)+Baff1;
        xlin1(:,i+1) = xlin1(:,i)+xdot1(:,i)*dt;
    end
    
    % Linearise with lambda = 0.5
    alpha = 0.5;
    xeq2 = findLinearizationPoint(xnonlin, x0, alpha);
    A2 = Asymfun(xeq2,0);
    Baff2 = Baffsymfun(xeq2,0);
    
    % Simulation of the linear model with lambda = 0.5
    xlin2 = x0;
    for i = 1:horizon
        xdot2(:,i) = A2*xlin2(:,i)+Baff2;
        xlin2(:,i+1) = xlin2(:,i)+xdot2(:,i)*dt;
    end
    
    % Update interactive plot
    delete(linPrediction)
    delete(linPrediction2)
    delete(nonlinPrediction)
    nonlinPrediction = plot(cutSwingPercentageData,xnonlin(1:2,:),'k','LineWidth',1.3);
    linPrediction = plot(cutSwingPercentageData,xlin1(1:2,:).','b--','LineWidth',1.7);
    linPrediction2 = plot(cutSwingPercentageData,xlin2(1:2,:).','r--','LineWidth',1.7);
    legend('\theta_1','\theta_2','\theta_1 nonlin','\theta_2 nonlin', ...
    '\theta_1 lin \alpha = 0','\theta_2 lin \alpha = 0', ...
    '\theta_1 lin \alpha = 0.5','\theta_2 lin \alpha = 0.5','location','southwest')
    axis([0,100,-0.7,0.5])
    
    % Update saved variables for next cycle
    setappdata(f,'linPrediction',linPrediction);
    setappdata(f,'linPrediction2',linPrediction2);
    setappdata(f,'nonlinPrediction',nonlinPrediction);
end