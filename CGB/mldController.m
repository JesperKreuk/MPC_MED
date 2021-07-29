function y = mldController(x,t,counter,Qy,yrefD, alpha)
    if t < 5
        y = [0;0];
    else
        dt = 0.01;
        L = 1;
        yref = 2*L*asin(yrefD/(2*L)); % compensation for the linearization
        
       
        if x(5) == 0
            counter = 0;
        elseif x(5) == 1
            counter = counter + 1;
        end
        kimp = 49-counter;
        if x(5) == 1
            if kimp > 0
                % Prosthetic swing
                
%                 UNCOMMENT THIS FOR ALPHA = 0.5 CONTROL
                tspan = 0:1e-3:0.55;
                
                load('CGBoptimalParameters.mat')
                m = parametersOpt(1);
                a = parametersOpt(2);
                mH = parametersOpt(3);
                phi1 = parametersOpt(4);
                phi2 = parametersOpt(5);
                [~,xnonlin] = ode45(@(t,x) dynamics(t,x(1:4),m,a,mH,phi1,phi2), tspan,x(1:4));
                xnonlin = xnonlin.';
                xeq = findLinearizationPoint(xnonlin, x(1:4), alpha);
                
%                 sw_cond = xnonlin(:,1)+xnonlin(:,2); % Switch condition th1+th2 = 0
%                 crossIdx = find(sw_cond(2:end).*sw_cond(1:end-1)<0);
%                 
%                 if isempty(crossIdx)
%                     crossIdx = 1;
%                 end
%                 crossIdx = crossIdx(end);
%                 xend = xnonlin(crossIdx,:).';
%                 xeq = x(1:4)*(1-alpha) + xend*alpha;

                Abarc = Asymfun(xeq,0);
                Bubarc = Bsymfun(xeq,0);
                Baffbarc =  Baffsymfun(xeq,0);

                S = buildMLD(Abarc, Bubarc, Baffbarc, dt);

                Qu = 1*eye(kimp+8);
                [H, f, constant, Acon, bcon, ivar, lb, ub] = buildMIQP(S, Qy, Qu, x, yref, kimp+8);
                
                % Setup Gurobi
                model = setupGurobi(H, f, constant, Acon, bcon, ivar, lb, ub);
                params_gurobi.outputflag = 0;

                % Optimize V = [u;w] with Gurobi
                result = gurobi(model, params_gurobi);
                if strcmp(result.status, 'OPTIMAL')
                    uoptvector = result.x;
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


    