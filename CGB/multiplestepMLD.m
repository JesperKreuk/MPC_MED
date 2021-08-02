%{
This function can make multiple simulation steps given the MLD system.

Arguments:
* S: Structure containing the MLD model
* x0: current state of the CGB model [th1;th2;dth1;dth2]
* u: input vector containing the torques of the MED
* N: number of steps to be taken by the function

Output: 
* x: A matrix containing the sequence of states 
* y: A vector containing the sequence of outputs (the ankle to ankle
    distance)
* flag: logical boolean, 1 if an MLD constraint is violated, 0 else
* w: A matrix containing the sequence of auxiliary variables

Author: Jesper Kreuk
%}

function [x,y,flag,w] = multiplestepMLD(S,x0,u,N)
    % Start the sequence with the current state and current output
    x = x0;
    y0 = S.C*x0;
    y = y0;
    w = [];

    % Start with no equality constraints are violated
    flag = false;
    for i = 1:N
        % For each time step get the next state
        [xnext, ynext, flagonestep, w0] = onestepMLD(S,x0,u(i));
        % And check if any of the constraints were violated in this step
        if flagonestep
            flag = true;
        end
        % Update all sequences
        w = [w, w0];
        x = [x, xnext];
        y = [y, ynext];
        x0 = xnext;
    end
end