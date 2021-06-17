function [u] = MPController_full(N,X_meas,X_set,dt)
%cascade MPC controllers

%weights for the position controller
Q_pos = diag([200 200 100 4 4 1]); %weights for [x y z vx vy vz]
R_pos = diag([10 10 2]);  %weights for output[phi_set theta_set thrust]


%weights for the angular controller
Q_ang = diag([300 300 1000 1 1 1]); %weights for [phi theta psi omega_phi omega_theta omega_psi]
R_ang = 10*eye(3);%weights for output[tau_phi tau_theta tau_psi]

%u_intermediate = [phi_set theta_set thrust]
u_intermediate = MPController_pos(Q_pos,R_pos,N,X_meas,X_set,dt);

%setting new desired angles phi and theta
X_set(7) = -u_intermediate(1);
X_set(8) = -u_intermediate(2);

%using angular controller to generate torques
torques = MPController_ang(Q_ang,R_ang,N,X_meas,X_set,dt);

%merging outputs into one array
u = [torques;u_intermediate(3)];

%normalizing output 
u = normalizeU(u);
end

