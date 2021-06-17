function [u] = pdController(Kp, Kd, X_set, X_actual, u_tilde)
% full controller
% only proportional-derivative controller because we already have the errors
% Kp = proportional gains
% Kd = derivative gains
% X_actual = measured state
% X_set = set state
% u_tilde = equilibrium input

    %error signals
	e = X_set - X_actual;
	e_x = e(1);
    e_y = e(2);
    e_z = e(3);
	e_vx = e(4);
    e_vy = e(5);
    e_vz = e(6);

	%acceleration outer controller (hand tuned)
    acc_x = -5.9e-2*e_x + -4e-1*e_vx;
    acc_y = -6e-2*e_y -4e-1*e_vy;
    acc_z = 1e-1*e_z + 1e-1*e_vz + 9.8;
    acc = [acc_x acc_y acc_z];
    
    %getting new angles to get to move to desired position
    X_set(7) = asin( (acc_x*sin(e(9)) - acc_y*cos(e(9))) / norm(acc) );
    X_set(8) = atan((acc_x*cos(e(9)) + acc_y*sin(e(9))) / acc_z);
    
	u = pdStabilizer(Kp, Kd, X_set, X_actual, u_tilde);

    u = normalizeU(u);    
end