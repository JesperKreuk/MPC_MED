%{
This function contains the nonlinear dynamics of the CGB model in
continuous time, it can calculate the state derivative.
The dynamics:
                M*ddth + C*dth+gbold = S*u

Arguments:
* t: current time, standard for ode45 functions, is not used
* x: current state of the CGB [th1;th2;dth1;dth2]
* m: leg mass in kg
* a: distance from the ankle to the CoM of the leg in m
* mH: hip mass in kg
* phi1: virtual gravity angle for the left (stance) leg theta1 in rad
* phi2: virtual gravity angle for the right (swing) leg theta2 in rad
* varargin: standard option, not used

Output: 
* dx: derivative of state x at the current time step
* y: output of the system, this is an approximation of the ankle to ankle 
    distance


Author: Jesper Kreuk
%}


function [dx, y] = dynamics(t, x, m, a, mH, phi1, phi2, varargin)
    % Define some constants
    g =9.81;    % gravitational acceleration in m/s/s
    L = 1;      % leg length in m
    b = L-a;    % distance from the CoM of a leg to the hip 
    
    % Extract angles and angular velocities from state x
    th1 = x(1);
    th2 = x(2);
    dth1 = x(3);
    dth2 = x(4);
    
    % Calculate torques from virtual gravity
    u_ank = (mH*L+m*L+m*a)*g*cos(th1)*tan(phi1)-m*b*g*cos(th2)*tan(phi2);
    u_hip = m*b*g*cos(th2)*tan(phi2);
    u = [u_ank;u_hip];

    % The equations of motion M*ddth + C*dth+gbold = S*u
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