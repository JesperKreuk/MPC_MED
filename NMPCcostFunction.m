function [fx, yend] = NMPCcostFunction(u, x0, Qy, Qu, kimp, yref, params)
    % uInit is the initial guess of the input sequense 
    %           [u(0),..., u(kimp-1)]
    % x0 is the known initial state x(0)
    % params contains parameters of the model
    a = params.a;
    L = params.L;
    b = params.b;
    m = params.m;
    mH = params.mH;
    g = params.g;
    phi1 = params.phi1;
    phi2 = params.phi2;
    dt = params.dt;
   
    x = zeros(4,kimp); % [x(1), ... x(kimp)]
    xk = x0;
    for i = 1:kimp
        if xk(1) + xk(2) >= 0
            x(:,i) = xk;
        else
    %         dx = CWodefunInput(t,xk,u(i),m,a,mH,phi1,phi2);
            k1 = dt*dynamicsNMPC(xk,u(i),m,a,mH,phi1,phi2);
            k2 = dt*dynamicsNMPC(xk+k1/2,u(i),m,a,mH,phi1,phi2);
            k3 = dt*dynamicsNMPC(xk+k2/2,u(i),m,a,mH,phi1,phi2);
            k4 = dt*dynamicsNMPC(xk+k3,u(i),m,a,mH,phi1,phi2);
            x(:,i) = xk + (k1+2*k2+2*k3+k4)/6;
        end
        xk = x(:,i);
    end
    xend = x(:,end);
    yend = sin(xend(1))-sin(xend(2));
    fx = (yend-yref)'*Qy*(yend-yref) + u'*Qu*u;
end