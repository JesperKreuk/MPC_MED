function S = buildMLD(Abarc,Bbarc,Baffbarc, dt)

    % Find linear matrices in continuous time
%     Abarc = double([zeros(2),eye(2), zeros(2,1);eval(Asym),zeros(2,1);zeros(1,5)]);
%     Bbarc = double([0;0;eval(Bsym);1]);
%     Baffbarc = double([0;0;eval(ddth_nonlin);0]);
    
%     % Add state gamma
%     Abarc = double([Abarc,zeros(4,1);zeros(1,5)]);
%     Bbarc = double([Bbarc;1]);
%     Baffbarc = double([Baffbarc;0]);
    
    % Find linear matrices in discrete time
    Abar = eye(4)+Abarc*dt;
    Bubar = Bbarc*dt;
    Baffbar = Baffbarc*dt;

    Mx = [pi/2;pi/2;100;100];  % max(xbar)
    mx = -[pi/2;pi/2;100;100]; % min(xbar)
    Mu = 0;                        % max(u)
    mu = -5;                       % min(u)
    L = 1;
    c = 0.001; % only able to switch if th1 >= c
    precision = eps;
    S.lb.x = mx;
    S.lb.u = mu;
    S.lb.aux = [mx;mu;0;0;0]; 
    S.ub.x = Mx;
    S.ub.u = Mu;
    S.ub.aux = [Mx;Mu;1;1;1];
    S.dt = dt;
    
    % Add state x_p 
    S.A = [eye(4), Baffbar;
        zeros(1,4), 1];

    S.Bu = zeros(5,1);

    S.Baux = [Abar-eye(4), Bubar, zeros(4,3);
            zeros(1,7), -1];

    S.Baff = zeros(5,1);

    S.C = [L, -L, zeros(1,3)];

    S.Daux = zeros(1,8);
    
    multiplier = 1e6;
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
        
    