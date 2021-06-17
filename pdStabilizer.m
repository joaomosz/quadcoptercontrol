function [u] = pdStabilizer(Kp, Kd, X_set, X_actual, u_tilde)
%stabilizes the drone at the desired angles and Z position
% only proportional-derivative controller because we already have the errors
% Kp = proportional gains
% Kd = derivative gains
% X_actual = measured state
% X_set = set state
% u_tilde = equilibrium input

	u = zeros(size(u_tilde));
	e = X_set - X_actual;
    
    %error signals
	e_phi = e(7);
	e_theta = e(8);
	e_psi = e(9);
	e_phi_dot = e(10);
	e_theta_dot = e(11);
	e_psi_dot = e(12);
	e_z = e(3);
	e_vz = e(6);
	
    %generate outputs with PD controller
	u(1) = e_phi*Kp(1) + e_phi_dot*Kd(1);
	u(2) = e_theta*Kp(2) +e_theta_dot*Kd(2);
	u(3) = e_psi*Kp(3) + e_psi_dot*Kd(3);
	u(4) = e_z*Kp(4) + e_vz*Kd(4);
	
    % apply equilibrium input
	u = u + u_tilde;

end