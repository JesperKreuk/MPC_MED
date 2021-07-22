function [x,y,flag,w] = multiplestepMLD(S,x0,u,N)
    % This function can only handle 1 input variable
    x = x0;
    y0 = S.C*x0;
    y = y0;
    w = [];

    flag = false;
    for i = 1:N
        [xnext, ynext, flagonestep, w0] = onestepMLD(S,x0,u(i));
        % If any flag from the onestep MLD = 1, return flag 1
        if flagonestep
            flag = true;
        end
        w = [w, w0];
        x = [x, xnext];
        y = [y, ynext];
        x0 = xnext;
    end
end