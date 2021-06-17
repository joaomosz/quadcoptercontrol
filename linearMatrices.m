function [ A,B,C,D ] = linearMatrices(X_tilde, u_tilde, type)
%creates the Jacobian matrices for linearized system around X_tilde
%X_dot = A*X + B*u
%y = C*X + D*u

%general parameters
m = 28e-3; %mass of crazifie (kg)
kd = 0; %drag coefficient (kg/s) negligeble for low speeds
I = 1e-6 * diag([16.571710, 16.655602, 29.261652]); %inertia matrix (kg*m^2)
g = 9.8; %earths gravity

%auxiliary variables
Ixx = I(1,1); Iyy = I(2,2); Izz = I(3,3);

%creating matrix A (Jacobian w.r.t X evaluated at X_tilde and u_tilde)
A = zeros(12);
A(1,4) = 1;

A(2,5) = 1;

A(3,6) = 1;

A(4,4) = -kd/m;
A(4,8) = -cos(X_tilde(8))*u_tilde(4)/m;

A(5,5) = -kd/m;
A(5,7) = cos(X_tilde(7))*cos(X_tilde(8))*u_tilde(4)/m;
A(5,8) = -sin(X_tilde(8))*sin(X_tilde(7))*u_tilde(4)/m;

A(6,6) = -kd/m; 
A(6,7) = -sin(X_tilde(7))*cos(X_tilde(8))*u_tilde(4)/m; 
A(6,8) = -sin(X_tilde(8))*cos(X_tilde(7))*u_tilde(4)/m;

A(7,7) = tan(X_tilde(8))*(cos(X_tilde(7))*X_tilde(11)-sin(X_tilde(7))*X_tilde(12));
A(7,8) = (sec(X_tilde(8))^2)*(sin(X_tilde(7))*X_tilde(11)+cos(X_tilde(7))*X_tilde(12));
A(7,10) = 1;
A(7,11) = sin(X_tilde(7))*tan(X_tilde(8));
A(7,12) = cos(X_tilde(7))*tan(X_tilde(8));

A(8,7) = -sin(X_tilde(7))*X_tilde(11) - cos(X_tilde(7))*X_tilde(12);
A(8,11) = cos(X_tilde(7));
A(8,12) = -sin(X_tilde(7));

A(9,7) = sec(X_tilde(8))*(cos(X_tilde(7))*X_tilde(11) - sin(X_tilde(7))*X_tilde(12));
A(9,8) = sec(X_tilde(8))*tan(X_tilde(8))*(sin(X_tilde(7))*X_tilde(11)+cos(X_tilde(7))*X_tilde(12));
A(9,11) = sin(X_tilde(7))*sec(X_tilde(8));
A(9,12) = cos(X_tilde(7))*sec(X_tilde(8));

A(10,11) = -(Iyy-Izz)/Ixx * X_tilde(12);
A(10,12) = -(Iyy-Izz)/Ixx * X_tilde(11);

A(11,10) = -(Izz-Ixx)/Iyy * X_tilde(12);
A(11,12) = -(Izz-Ixx)/Iyy * X_tilde(10);

A(12,10) = -(Ixx-Iyy)/Izz * X_tilde(11);
A(12,11) = -(Ixx-Iyy)/Izz * X_tilde(10);

%creating matrix B (Jacobian w.r.t u evaluated at X_tilde and u_tilde)
B = zeros(12,4);
B(4,4) = -sin(X_tilde(8))/m;
B(5,4) = cos(X_tilde(8))*sin(X_tilde(7))/m;
B(6,4) = cos(X_tilde(8))*cos(X_tilde(7))/m;
B(10,1) = 1/Ixx;
B(11,2) = 1/Iyy;
B(12,3) = 1/Izz;

%creating matrix C (output part based on state)
switch type
    case 'half'
        C = zeros(6,12);
        C(1,1)=1;
        C(2,2)=1;
        C(3,3)=1;
        C(4,7)=1;
        C(5,8)=1;
        C(6,9)=1;
        D = zeros(6,4); %no throughput
    case 'full'
        C = eye(12); %fully observed!
        D = zeros(12,4); %no throughput
    otherwise
        error('invalid observation type!');
end

end

