function [ X_new ] = eulerSolver( X_old, X_dot, dt )
%numerically solver differential equation using euler method
    
    X_new = X_old + X_dot * dt;
end

