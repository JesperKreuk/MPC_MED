function y = nonlinController(x,t,counter,Qy,yref)
    if t < 5
        y = [0;0];
    else
        dt = 0.01;
        controlParams.Qy = Qy;
        controlParams.yref = yref;
        load('CGBoptimalParameters.mat')
        params = setupParamsStruct(parametersOpt, dt, controlParams);
        if x(5) == 0
            counter = 0;
        elseif x(5) == 1
            counter = counter + 1;
        end
        kimp = 49-counter;
        if x(5) == 1
            if kimp > 0
                % Prosthetic swing
                uoptvector = NMPC(x(1:4), kimp+8, params);
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


    