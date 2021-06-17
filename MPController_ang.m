function [u] = MPController_ang(Q,R,N,X_meas,X_set,dt)
%uses positions subsystem linearized model to generate torques [tau_phi tau_theta tau_psi]
%Q =  weights for [phi theta psi omega_phi omega_theta omega_psi]
%R = weights for output[tau_phi tau_theta tau_psi]
%X_meas = measured state
%X_set = set state

%getting only angular values from state
X_meas = X_meas(7:12);
X_set = X_set(7:12);

%creating linearized matrices for angular subsystem
A = zeros(6);
A(1:3,4:6) = eye(3);
B = zeros(6,3);
B(4:6,1:3)=eye(3);
%discretized matrices
A = eye(6) + dt*A;
B = dt*B;

%Creating extended matrices for N future predictions
    A_hat = zeros(6*N,6);
    B_hat = zeros(6*N,3*(N-1));
    Q_hat = zeros(6*N,6*N);
    R_hat = zeros(3*(N-1),3*(N-1));
    X_set_aug = zeros(6*N,1);
%Populating above said matrices
for i=1:N
    A_hat(6*(i-1)+1:6*i,1:6) = A^(i);
    Q_hat(6*(i-1)+1:6*i,6*(i-1)+1:6*i) = Q;
    X_set_aug(6*(i-1)+1:6*i) = X_set;
    if i<N
        R_hat(3*(i-1)+1:3*i,3*(i-1)+1:3*i) = R;
    end
    for j = 1:N-1
        if j<=i
            B_hat(6*(i-1)+1:6*i,3*(j-1)+1:3*j) = A^(i-1)*B;
        end
    end
end

%auxiliary variables
H_hat = B_hat.'*Q_hat*B_hat + R_hat;
c = (Q_hat*B_hat).' * (A_hat*X_meas - X_set_aug);

%quadratic optimization solution
u_star = -inv(H_hat)*c;
u = u_star(1:3);

end

