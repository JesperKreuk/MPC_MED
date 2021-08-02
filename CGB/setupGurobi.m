%{
This function sets up the gurobi function that uptimizes the MIQP

Arguments:
* H: Constant matrix for quadratic terms of V
* f: Constant vector for linear terms of V
* constant: the constant of the cost function that is not a function of V 
* Acon: Constant matrix of constraint equation Acon*V = bcon 
* bcon: Constant vector of constraint equation Acon*V = bcon 
* ivar: logical vector where 1 means the corresponding element of V is
    binary
* lb: lower bound of V
* ub: upper bound of V

Output: 
* model: the model that is optimized by Gurobi

Author: Jesper Kreuk
%}

function model = setupGurobi(H, f, constant, Acon, bcon, ivar, lb, ub)
    vtype = repmat('C',size(ivar));
    vtype(logical(ivar)) = 'B';
    model.A = sparse(Acon); % Constraint matrix, has to be sparse
    model.obj = f.'; % linear objective function
    model.objcon = constant; % the constant term of the objective function
    model.Q = sparse(H); % Quadratic objective function
    model.rhs = bcon; % right hand side vector
    model.sense = '<'; % Constraint sense vector
    model.vtype = vtype; 
    model.lb = lb;
    model.ub = ub;
end