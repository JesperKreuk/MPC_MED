function y = nonlinController(x,t,counter)
%     coder.extrinsic('NMPC_mex'); 
    if t < 5
        y = [0;0];
    else
        dt = 0.01;
        Qy = 1e8;
        yref = 0.7;
        load('Optimal_params_30percentSong.mat')
        params = setupParamsStruct(ParametersOpt, Qy, yref, dt);
        if x(5) == 0
            counter = 0;
        elseif x(5) == 1
            counter = counter + 1;
        end
        kimp = 49-counter;
        if x(5) == 1
            dt = params.dt;
    %         timp = heelstrike(x(1:4),params); % Find when heelstrike occurs
    %         kimp = round(timp/dt);
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


    