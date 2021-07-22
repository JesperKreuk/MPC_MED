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