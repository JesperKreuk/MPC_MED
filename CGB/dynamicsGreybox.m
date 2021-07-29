function [dx, y] = dynamicsGreybox(t, x, u, m, a, mH, phi1, phi2, varargin)
    g =9.81;
    L = 1;
    b = L-a;

    th1 = x(1);
    th2 = x(2);
    dth1 = x(3);
    dth2 = x(4);
    
    u1 = (mH*L+m*L+m*a)*g*cos(th1)*tan(phi1)-m*b*g*cos(th2)*tan(phi2);
    u2 = m*b*g*cos(th2)*tan(phi2);
    u = [u1;u2];

    M = [mH*L^2+m*a^2+m*L^2, -m*b*L*cos(th1-th2);
        -m*b*L*cos(th1-th2), m*b^2];
    C = [0, -m*b*L*sin(th1-th2)*dth2;
        m*b*L*sin(th1-th2)*dth1,0];
    gbold = [-(mH*L+m*a+m*L)*sin(th1);
        m*b*sin(th2)]*g;

    S = [1 1; 0 -1];
    
    dx=zeros(4,1);
    dx(1)=x(3);
    dx(2)=x(4);
    dx(3:4)=M\(S*u-C*[dth1;dth2]-gbold);
    y = [x(1);x(2)];
end