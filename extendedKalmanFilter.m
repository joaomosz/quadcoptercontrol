function [X_estimated, P_estimated] = extendedKalmanFilter(P_init, X_init,u_init, dt, R, Q, Y_meas, type)
%Produce a state estimate using the extended kalman filter
%P = signal covariance matrix
%R = noise measurement covariance matrix
%Q = disturbance covariance matrix
%X_init = previously filtered state
%u_init = previous input
%Y_meas = current measured state
%type = observation type (full or half)

    X_dot = quadcopterSystem(u_init, X_init);
    
    %linearize around current point
    [ F,~,H,~ ] = linearMatrices(X_init, u_init, type);
    P_dot = F.' * P_init + P_init*F + Q;
    
    %update estimates of signal and signal covariance matrix
       P_hat = P_init + P_dot*dt;
       X_hat = X_init + X_dot*dt;
    
   %calculate filter coefficient
   K = P_hat * H.' * inv(H*P_hat*H.' + R);
   
   %improve estimate based on observation
   X_estimated = X_hat + K*(Y_meas - H*X_hat);
   P_estimated = P_hat - K*H*P_hat;
end

