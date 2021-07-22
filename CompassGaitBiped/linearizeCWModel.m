function [Asym,Bsym,Baffsym,ddth_nonlin] = linearizeCWModel(params)
    % This function calculates variables
    % * Asym
    % * Bsym
    % * Baffsym
    % That describe the system dx(t) = Asym*x(t)+Bsym*u(t)+Baffsym
    %                                   (continuous time)
    % Both are used for the linearization in a current timestep, the
    % linearization is based on a nonlinear model that is estimated by a grey
    % box estimation method from the file 'CWgreyest.m'
    
    
    %%% Parameters
    L = params.L;
    m = params.m;
    a =  params.a;
    mH =  params.mH;
    phi1 =  params.phi1;
    phi2 = params.phi2;
    b = params.b;
    g = params.g;

    syms th1 th2 dth1 dth2 CMGangle uCMG
    th = [th1;th2];
    dth = [dth1;dth2];

    M = [mH*L^2+m*a^2+m*L^2, -m*b*L*cos(th1-th2);
        -m*b*L*cos(th1-th2), m*b^2];
    C = [0, -m*b*L*sin(th1-th2)*dth2;
        m*b*L*sin(th1-th2)*dth1,0];
    gbold = [-(mH*L+m*a+m*L)*sin(th1);
        m*b*sin(th2)]*g;
    S = [1 1 0;0 -1 1]; % This gives a torque uCMG on theta2
    u1 = (mH*L+m*L+m*a)*g*cos(th1)*tan(phi1)-m*b*g*cos(th2)*tan(phi2);
    u2 = m*b*g*cos(th2)*tan(phi2);
%     u3 = 2*Iws*Omega*cos(CMGangle)*uCMG;
    u = [u1;u2; uCMG]; % uCMG is torque input from the CMG


    ddth_nonlin = M\(-C*dth-gbold+S*u);

    
    Asym = [zeros(2),eye(2);simplify(jacobian(ddth_nonlin,[th;dth]))];
    Bsym = [zeros(2,1);simplify(jacobian(ddth_nonlin,uCMG))];
    Baffsym = [dth1;dth2;ddth_nonlin]-Asym*[th;dth]-Bsym*uCMG;
end
