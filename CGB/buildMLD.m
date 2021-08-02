%{
This function writes the linear MLD system of the CGB model given the 
linear continuous time matrices such the MLD system has the following form
           x(k+1) = A*x(k) + Bu*u + Baux*w + Baff
           y(k) = C*x(k) 
For more details, see Appendix D of the corresponding thesis

Arguments:
* Ac: Constant square state matrix from dx = Ac*x+Bc*u + Baffc (continuous-time)
* Bc: Constant input matrix from dx = Ac*x+Bc*u + Baffc (continuous-time)
* Baffc: Constant affine term from dx = Ac*x+Bc*u + Baffc (continuous-time)
* dt: step time in seconds

Output: 
* S: Structure containing the MLD model

Author: Jesper Kreuk
%}

function S = buildMLD(Ac,Bc,Baffc, dt)    
    % Find linear matrices in discrete time, use a zero order hold
    A = eye(4)+Ac*dt;
    Bu = Bc*dt;
    Baff = Baffc*dt;
    
    % Some constants used
    L = 1;
    c = 0.001; % only able to switch if th1 >= c
    precision = eps; % Machine precision 

    % Define lower and upper bounds
    Mx = [pi/2;pi/2;100;100];  % max(xbar)
    mx = -[pi/2;pi/2;100;100]; % min(xbar)
    Mu = 0;                        % max(u)
    mu = -5;                       % min(u)
    S.lb.x = mx;
    S.lb.u = mu;
    S.lb.aux = [mx;mu;0;0;0]; 
    S.ub.x = Mx;
    S.ub.u = Mu;
    S.ub.aux = [Mx;Mu;1;1;1];
    
    % Add the time step to the structure
    S.dt = dt;
    
    % Define the MLD with augmented state vector xbar = [x; state x_p]
    S.A = [eye(4), Baff;
        zeros(1,4), 1];

    S.Bu = zeros(5,1);

    S.Baux = [A-eye(4), Bu, zeros(4,3);
            zeros(1,7), -1];

    S.Baff = zeros(5,1);

    S.C = [L, -L, zeros(1,3)];
    
    % Define constraint matrices
    % Ex*x + Eu*u + Eaux*w <= Eaff
    
    multiplier = 1e6; % A multiplier is used because some elements are 
    % otherwise too close to zero
    S.Ex = multiplier*[-1 0 0 0 0;
          1 0 0 0 0;
          -1 -1 0 0 0;
          1 1 0 0 0;
          0 0 0 0 -1;
          0 0 0 0 1;
          zeros(4) -Mx;
          zeros(4) mx;
          -eye(4) -mx;
          eye(4) Mx;
          zeros(1,4) -Mu;
          zeros(1,4) mu;
          zeros(1,4) -mu;
          zeros(1,4) Mu];

    S.Eu = multiplier*[0;
            0;
            0;
            0;
            0;
            0;
            zeros(4,1);
            zeros(4,1);
            zeros(4,1);
            zeros(4,1);
            0;
            0;
            -1;
            1];

    S.Eaux = multiplier*[zeros(1,5) pi/2 0 0;
            zeros(1,5) -pi/2-precision 0 0;
            zeros(1,5) 0 pi 0;
            zeros(1,5) 0 -pi-precision 0;
            zeros(1,5) -1 -1 4;
            zeros(1,5) 1 1 -1-precision;
            eye(4) zeros(4,4);
            -eye(4) zeros(4,4);
            eye(4) zeros(4,4);
            -eye(4) zeros(4,4);
            zeros(1,4) 1 0 0 0;
            zeros(1,4) -1 0 0 0;
            zeros(1,4) 1 0 0 0;
            zeros(1,4), -1 0 0 0];

    S.Eaff = multiplier*[pi/2-c;
              -precision+c;
              pi;
              -precision;
              1.5;
              -precision+2.5;
              zeros(4,1);
              zeros(4,1);
              -mx;
              Mx;
              0;
              0;
              -mu;
              Mu];
          
    S.j.d = [6 7 8]; % The 6th, 7th and 8th auxiliary variable is are binary
end
        
    