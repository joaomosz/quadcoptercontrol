function [ Xdot ] = quadcopterSystem(u, X)
%returns the derivatives of the crazyflie system based in input U and current state X

%x_dot = f(X,u)

% Quadcopter parameters
%the center of mass is at the geometric center in xy plane(looking drone
%from above. The z coordinate is at 17.425mm above the ground when
%supported by a surface
%Size (WxHxD): 92x92x29mm (motor-to-motor and including motor mount feet)
m = 28e-3; %mass of crazifie (kg) 
kd = 0; %drag coefficient (kg/s) negligeble for low speeds
I = 1e-6 * diag([16.571710, 16.655602, 29.261652]); %inertia matrix (kg*m^2)
g = 9.8; %earths gravity


%output
Xdot = zeros(size(X));

%auxiliary
lin_vel = X(4:6);
angles = X(7:9);
omegas = X(10:12);

Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

crossProd = [(Iyy - Izz)*omegas(2)*omegas(3)/Ixx;...
             (Izz - Ixx)*omegas(1)*omegas(3)/Iyy;...
             (Ixx - Iyy)*omegas(1)*omegas(2)/Izz ];

invI = 1./[Ixx Iyy Izz];

%calculating output

Xdot(1:3) = lin_vel;

Xdot(4:6) = [0; 0; -g] + (1/m)*RotationMatrix(angles)*([0 0 u(4)].')- (kd/m)*lin_vel;

Xdot(7:9) = InvAngleRatesMatrix(angles)*omegas;

Xdot(10:12) = diag(invI)*[u(1);u(2);u(3)] - crossProd;

end

