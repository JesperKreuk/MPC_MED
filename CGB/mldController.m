%{
This is the mld controller called by simulink. It calculates the optimal
torque of the MED based on the linearised dynamics of the CGB model. 

Arguments:
* xbar: the current augmented state xbar = [x;xp] 
* t: the current time since the start of the simulation
* counter: counts how many time steps the model is in prosthetic swing
* Qy: A weight matrix that penalizes if the predicted foot location differs
    from the reference
* yrefD: the desired reference, this reference is compensated because the
    MLD uses a linear approximation
* alpha: a constant used to calculate the linearization point

Output: 
* y: a vector containing the optimal torque of the MED and the updated 
    counter [u_med; counter]

Author: Jesper Kreuk
%}

function y = mldController(xbar,t,counter,Qy,yrefD, alpha)    
    if t < 5
        % The controller is not active for the first 5 seconds
        y = [0;0];
    else
        % After 5 seconds the controller is activated
        
        % Set some constants
        dt = 0.01;
        L = 1;
        yref = 2*L*asin(yrefD/(2*L)); % compensation for the linearization
       
        % Update the counter, this counter keeps track of how many previous
        % time steps were in prostetic swing
        if xbar(5) == 0
            counter = 0;
        elseif xbar(5) == 1
            counter = counter + 1;
        end
        % From trail and error it is found that a kimp of 49 is
        % sufficiently large
        kimp = 49-counter;
        if xbar(5) == 1
            if kimp > 0
                % Prosthetic swing
                tspan = 0:1e-3:0.55;
                
                % Load the parameters of the CGB model
                load('CGBoptimalParameters.mat')
                m = parametersOpt(1);
                a = parametersOpt(2);
                mH = parametersOpt(3);
                phi1 = parametersOpt(4);
                phi2 = parametersOpt(5);
                
                % Simulate the nonlinear CGB model to approximate the final
                % state
                [~,xnonlin] = ode45(@(t,x) dynamics(t,x(1:4),m,a,mH,phi1,phi2), tspan,xbar(1:4));
                xnonlin = xnonlin.';
                
                % Find the linearization point
                xeq = findLinearizationPoint(xnonlin, xbar(1:4), alpha);
                
                % Linearize the CGB model
                Ac = Asymfun(xeq,0);
                Buc = Bsymfun(xeq,0);
                Baffc =  Baffsymfun(xeq,0);
                
                % Build the corresponding MLD system
                S = buildMLD(Ac, Buc, Baffc, dt);
                
                % Find the matrices and vectors of MIQP problem in the 
                % format such that Gurobi can solve it
                Qu = 1*eye(kimp+8);
                [H, f, constant, Acon, bcon, ivar, lb, ub] = buildMIQP(S, Qy, Qu, xbar, yref, kimp+8);
                
                % Setup Gurobi
                model = setupGurobi(H, f, constant, Acon, bcon, ivar, lb, ub);
                params_gurobi.outputflag = 0;

                % Optimize V = [u;w] with Gurobi
                result = gurobi(model, params_gurobi);
                if strcmp(result.status, 'OPTIMAL')
                    uoptvector = result.x;
                    
                    % Only use the first input of the optimized vector
                    uopt = uoptvector(1);
                    if uopt > 0
                        uopt;
                        uopt = 0;
                    end
                else
                    uopt = 0;
                end
            else
                uopt = 0;
            end
        else
            uopt = 0;
            kimp = 0;
        end
        y = zeros(2,1);
        y(1) = uopt;
        y(2) = counter;
    end
end


    