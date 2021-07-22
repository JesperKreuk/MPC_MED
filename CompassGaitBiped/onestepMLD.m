function [x1, y1, flag, w0] = onestepMLD(S,x0,u0)
    c = 0.001; % only able to switch if th1 >= c
    z1 = x0(1:4)*x0(5);
    z2 = u0*x0(5);
    if x0(1) >= c
        deltaLS = 1;
    else
        deltaLS = 0;
    end
    if x0(1)+x0(2) >= 0
        deltaimp = 1;
    else
        deltaimp = 0;
    end
    if deltaimp+deltaLS+x0(5) >= 3
        deltaSW = 1;
    else
        deltaSW = 0;
    end
    w0 = [z1;z2;deltaLS; deltaimp;deltaSW];
    x1 = S.A*x0+S.Bu*u0+S.Baux*w0+S.Baff;
    y1 = S.C*x1;
    
    check_conditions = S.Ex*x0+S.Eu*u0+S.Eaux*w0 <= S.Eaff;
%     S.Ex(21,:)*x0+S.Eu(21,:)*u0+S.Eaux(21,:)*w0 <= S.Eaff(21,:)
    flag = any(~check_conditions); 
    % Flag = 1 means at least one inequality is bad
end