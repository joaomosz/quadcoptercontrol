function [x_hat] = kalmanFilter(y, u, x_hat_old,A,B,C,Kf,dt)
%kalman filter to produce state estimate from noisy and/or partially observed state y

%y = current measured noisy and/or partially state
%u = current input
%x_hat_old = last kalman state estimate
%A,B,C = state space matrices description
%Kf = kalman matrix filter

    x_noisy_dot = (A-Kf*C)*x_hat_old + Kf*y + B*u;
    x_hat = eulerSolver(x_hat_old, x_noisy_dot,dt);
end

