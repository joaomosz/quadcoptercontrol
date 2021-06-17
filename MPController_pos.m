function [u] = MPController_pos(Q,R,N,X_meas,X_set,dt)
%uses positions subsystem linearized model to generate [phi_set theta_set thrust]
%Q = weights for [x y z vx vy vz]
%R = weights for output[phi_set theta_set thrust]
%X_meas = measured state
%X_set = set state

%parameters
m = 28e-3; %mass of crazifie (kg)
g = 9.8; %earths gravity

%trimming positions values from state
X_meas = X_meas(1:6);
X_set = X_set(1:6);

%creating linearized matrices for position subsystem
A = zeros(6);
A(1:3,4:6) = eye(3);
B = zeros(6,3);
B(5,1)=-g;
B(4,2)=g;
B(6,3)=1/m;
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

%adding linearized input to the thrust 
u_star(3) = u_star(3) + m*g;

u = u_star(1:3);

end

