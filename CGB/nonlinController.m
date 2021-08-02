%{
This is the NMPC controller called by simulink. It calculates the optimal
torque of the MED based on the nonlinear dynamics of the CGB model. 

Arguments:
* xbar: the current augmented state xbar = [x;xp] 
* t: the current time since the start of the simulation
* counter: counts how many time steps the model is in prosthetic swing
* Qy: A weight matrix that penalizes if the predicted foot location differs
    from the reference
* yref: the ankle to ankle reference at heel strike

Output: 
* y: a vector containing the optimal torque of the MED and the updated 
    counter [u_med; counter]

Author: Jesper Kreuk
%}


function y = nonlinController(x,t,counter,Qy,yref)
    if t < 5
        % The controller is not active for the first 5 seconds
        y = [0;0];
    else
        % After 5 seconds the controller is activated, set some constants
        dt = 0.01; % Time step of the controller
        controlParams.Qy = Qy;
        controlParams.yref = yref;
        
        % Setup parameter struct
        load('CGBoptimalParameters.mat')
        params = setupParamsStruct(parametersOpt, dt, controlParams);
        
        % Update the counter, this counter keeps track of how many previous
        % time steps were in prostetic swing
        if x(5) == 0
            counter = 0;
        elseif x(5) == 1
            counter = counter + 1;
        end
        kimp = 49-counter;
        if x(5) == 1
            if kimp > 0
                % Prosthetic swing
                Qy = params.Qy;
                Qu = 1*eye(kimp+8);
                yref = params.yref;
                
                % Define cost function
                fun = @(u)NMPCcostFunction(u, x(1:4), Qy, Qu, kimp+8, yref, params);
                
                % Set upper and lower bounds
                lb = -5*ones(kimp+8,1);
                ub = 0*ones(kimp+8,1);
                uInit = zeros(kimp+8,1);
                
                % Optimize med torques
                opts = optimoptions('fmincon','Algorithm','sqp',...
                'OptimalityTolerance',1e-5);
                uoptvector = fmincon(fun,uInit,[],[],[],[],lb,ub,[],opts);
                
                % Execute first torque 
                uopt = uoptvector(1);
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


    