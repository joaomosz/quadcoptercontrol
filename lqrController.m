function [ u ] = lqrController(K, X_meas, X_set, u_set)
%generates the input using LQR algorithm u=-KX
    u = -K*(X_meas - X_set);
    % apply equilibrium input
	u = u + u_set;
	
    u = normalizeU(u);
end

