%{
This function find all matrices required for optimization of the MIQP
problem. The following form is made, for further details see Appendix E of
the corresponding thesis.
   min V'HV+f'V    subject to:   Acon*V <= bcon and lb <= V <= ub
    V
Where V = [u;w] is a vector with all inputs and aux variables, which can
be binary.


Arguments:
* S: Structure containing the MLD model
* Qy: Weight matrix that penalises the states
* Qu: Weight matrix that penalises the input
* x0: State of the current time step [th1;th2;dth1;dth2]
* yref: Reference, this is the ankle to ankle distance in m
* N: Prediction horizon (integer)

Output: 
* H: Constant matrix for quadratic terms of V
* f: Constant vector for linear terms of V
* constant: the constant of the cost function that is not a function of V 
* Acon: Constant matrix of constraint equation Acon*V = bcon 
* bcon: Constant vector of constraint equation Acon*V = bcon 
* ivar: logical vector where 1 means the corresponding element of V is
    binary
* lb: lower bound of V
* ub: upper bound of V

Author: Jesper Kreuk
%}

function [H, f, constant, Acon, bcon, ivar, lb, ub] = buildMIQP(S, Qy, Qu, x0, yref, N)
    % Extract MLD matrices and vectors
    A = S.A;
    Bu = S.Bu;
    Baux = S.Baux;
    Baff = S.Baff;
    C = S.C;
    Ex = S.Ex;
    Eu = S.Eu;
    Eaux = S.Eaux;
    Eaff = S.Eaff;
    
    % Define sizes
    nx = size(A,1);         % Number of states
    nu = size(Bu,2);        % Number of inputs
    ny = size(C,1);         % Number of outputs
    naux = size(Baux,2);    % Number of auxiliary variables
    
    ubin = zeros(nu,1); % There are no binary inputs
    
    % Define which auxiliary variables are binary
    wbin = zeros(naux,1);
    wbin(S.j.d) = 1;
    
    % Build T1 =[I;A;A^2;....] 
    T1 = zeros(N*nx,nx);
    for i = 1:N
        T1(1+nx*(i-1):nx*i,:) = A^(i-1);
    end

    % Build T2 = [0,        0,      0, ...;
    %             Bu,       0,      0, ...;
    %             A*Bu,     Bu,     0, ...;
    %             A^2*Bu,   A*Bu,   Bu, ...;
    %               ...                     ]
    T2 = zeros(N*nx,N*nu);
    for j = 1:N
        for i = 1:N
            if i-j-1 <0
                continue
            else
                T2(1+nx*(i-1):nx*i,1+nu*(j-1):nu*j) = A^(i-j-1)*Bu;
            end
        end
    end

    % Build T3 = [0,          0,      0, ...;
    %             Baux,       0,      0, ...;
    %             A*Baux,     Baux,     0, ...;
    %             A^2*Baux,   A*Baux,   Baux, ...;
    %               ...                     ]
    T3 = zeros(N*nx,N*naux);
    for j = 1:N
        for i = 1:N
            if i-j-1 <0
                continue
            else
                T3(1+nx*(i-1):nx*i,1+naux*(j-1):naux*j) = A^(i-j-1)*Baux;
            end
        end
    end

    % Build T4 = [0; Baff; (A+I)*Baff; (A^2+A+I)*Baff; ...];
    T4 = zeros(N*nx,1);
    for i = 1:N
        my_sum = zeros(nx);
        if i == 1
            my_sum = zeros(nx);
        else
            for j = 2:i
                my_sum = my_sum+A^(j-2);
            end
        end
        T4(1+nx*(i-1):nx*i,:) = my_sum*Baff;
    end

    % Build R1
    R1 = A^N;

    % Build R2 =[..., A^2*Bu, A*Bu, Bu] 
    R2 = zeros(nx,N*nu);
    for i = 1:N
        R2(:,1+nu*(i-1):nu*i) = A^(N-i)*Bu;
    end

    % Build R3 =[..., A^2*Baux, A*Baux, Baux] 
    R3 = zeros(nx,N*naux);
    for i = 1:N
        R3(:,1+naux*(i-1):naux*i) = A^(N-i)*Baux;
    end

    % Build R4 = [(...+A^2+A+I)*Baff]
    my_sum = zeros(nx);
    for i = 1:N
        my_sum = my_sum+A^(i-1);
    end
    R4 = my_sum*Baff;

    % Build Eps1 = diag([Ex,Ex,..])
    Eps1 = kron(eye(N),Ex);

    % Build Eps2 = diag([Eu,Eu,..])
    Eps2 = kron(eye(N),Eu);

    % Build Eps3 = diag([Eaux,Eaux,..])
    Eps3 = kron(eye(N),Eaux);

    % Build Eps4 = [Eaff;Eaff;...]
    Eps4 = repmat(Eaff,[N,1]);
    
    ivar = [];
    for i = 1:N
        ivar = [ivar;ubin];
    end
    for i = 1:N
        ivar = [ivar;wbin];
    end
    %% The optimization problem 
    %    min V'HV+f'V    subject to:   Acon*V <= bcon and lb <= V <= ub
    %     V
    H = [R2'*C'*Qy*C*R2+Qu, R2'*C'*Qy*C*R3;
           R3'*C'*Qy*C*R2, R3'*C'*Qy*C*R3];

    f = 2*[(x0'*R1'*C'+R4'*C'-yref')*Qy*C*R2, (x0'*R1'*C'+R4'*C'-yref')*Qy*C*R3].';

    constant = x0'*R1'*C'*Qy*C*R1*x0 + R4'*C'*Qy*C*R4+yref'*Qy*yref+ ...
        2*x0'*R1'*C'*Qy*C*R4-2*x0'*R1'*C'*Qy*yref-2*R4'*C'*Qy*yref;
    
    Acon = double([Eps2+Eps1*T2,Eps3+Eps1*T3]);
    bcon = double(Eps4-Eps1*T1*x0-Eps1*T4);

    lb = [repmat(S.lb.u,[N,1]);
            repmat(S.lb.aux,[N,1])];
    ub = [repmat(S.ub.u,[N,1]);
            repmat(S.ub.aux,[N,1])];
end
 


