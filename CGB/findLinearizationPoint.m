function xeq = findLinearizationPoint(xnonlin, x0, alpha)
    % Heel strike condition th1+th2 = 0
    heelstrike = xnonlin(1,:)+xnonlin(2,:); 
    
    % Find heelstrike index 
    crossIdx = find(heelstrike(2:end).*heelstrike(1:end-1)<0);
    if isempty(crossIdx)
        crossIdx = 1;
    end
    crossIdx = crossIdx(end);

    % Find equilibrium point
    xeq = x0*(1-alpha) + xnonlin(:,crossIdx)*alpha;
end
