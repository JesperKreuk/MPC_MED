function plotLines(src,event)
    % This is the callback function that is called when the slider is moved
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
    
    x0 = Rdatax(k,:).';
        
    % Nonlinear prediction
    m = ParametersOpt(1);
    a =  ParametersOpt(2);
    mH =  ParametersOpt(3);
    phi1 =  ParametersOpt(4);
    phi2 =  ParametersOpt(5);
    
    tspan = 0:dt:0.6;
    N = length(tspan);
    % Simulate system
    [~,xnonlin] = ode45(@(t,x)dynamics(t,x,m,a,mH,phi1,phi2), tspan, x0);
    xnonlin = xnonlin.';
    
    cutSwingPercentageData = mapfun(k:10:k+N*10-1, 1, Ndata,0,100);
    
    xeq1 = x0;
    A1 = Asymfun(xeq1,0);
    Baff1 = Baffsymfun(xeq1,0);
    
    xlin1 = x0;
    for i = 1:horizon
        xdot1(:,i) = A1*xlin1(:,i)+Baff1;
        xlin1(:,i+1) = xlin1(:,i)+xdot1(:,i)*dt;
    end
    
    alpha = 0.5;
    xeq2 = findLinearizationPoint(xnonlin, x0, alpha);

    A2 = Asymfun(xeq2,0);
    Baff2 = Baffsymfun(xeq2,0);
    
    xlin2 = x0;
    for i = 1:horizon
        xdot2(:,i) = A2*xlin2(:,i)+Baff2;
        xlin2(:,i+1) = xlin2(:,i)+xdot2(:,i)*dt;
    end
    
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
    setappdata(f,'linPrediction',linPrediction);
    setappdata(f,'linPrediction2',linPrediction2);
    setappdata(f,'nonlinPrediction',nonlinPrediction);
end