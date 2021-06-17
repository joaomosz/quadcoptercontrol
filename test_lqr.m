% Tests the Craziflie model using LQR and full state observation
% ONLY WORKS FULLY OBSERVED!

%performs well. High penalty for height, roll and pitch angles

%clc;clear all;close all;

noise = 0;
disturbance = 0;
observation = 'full'; %'full' or 'half'
%full = fully observed all 12 variables
%half = only measured position and angles (6 variables)

%% Linearization around equilibrium
% equilibrium state
X_tilde = [0 0 0 0 0 0 0 0 0 0 0 0].'; %[x,y,z,vx,vy,vz,phi,theta,psi,omega_phi,omega_theta,omega_psi]

%equilibrium input
u_tilde = equilibriumInput(X_tilde);

%creating linear matrices for linearized system
[ A,B,C,D ] = linearMatrices(X_tilde, u_tilde, observation);

% IMPORTANT:  matrix A in this conditions does not depend on:
%x y z vx vy vz psi

%% Controlability evaluation and controller creation
Ctr = ctrb(A,B); %controlability matrix
if rank(Ctr) == length(X_tilde) %check controlability condition
    disp('System is controllable around equilibrium point')
    %hand tune weights
    Q = diag([1 1 100 1 1 1 1000 1000 1 1 1 1]); %weights for [x,y,z,vx,vy,vz,phi,theta,psi,omega_phi,omega_theta,omega_psi]
    R = 1e6*eye(4); %weights for [tau_phi, tau_theta, tau_psi, thrust]
    K = lqr(A,B,Q,R);
else
    disp ('System is NOT controllable around equilibrium point')
end

%% Setting variables for simulation

% initial conditions
X = [0 0 0 0 0 0 0 0 0 0 0 0].'; %full state
u = [0 0 0 0].'; % input [tau_phi, tau_theta, tau_psi, thrust]
%changes in +phi -> changes in +Y
%changes in +theta -> changes in -X
%changes in psi -> no changes in coordinates (pan)

%auxliliary for intial conditions
X_true = X; %true = initial state (already known)

%set state
X_set = [0 0 0 0 0 0 0 0 30*pi/180 0 0 0].';
%simulation times
dt = 0.01; %step size
t_end = 40;
t = 0:dt:t_end; %simulation times
score = 0;

%creating array with states history for plotting purposes
True = zeros(12,length(t)); %true (noiseless) results
Observed = zeros(size(C,1),length(t)); % noisy states

%covariance matrices
Wn = noise*eye(size(C,1)); %measurement noise covariance matrix
Wd = disturbance*eye(12); %disturbance in state covariance matrix

%% loop to solve differential equation
tic
for i=1:length(t)
    True(:,i) = X_true; %populating state in result array
    
    %true system
    X_dot = quadcopterSystem(u, X);
    X_true = eulerSolver(X_true, X_dot, dt); 
    
    %getting imperfect next state (derivatives with noise)
    X = eulerSolver(X, X_dot + Wd*randn(12,1), dt); %getting next state 
    
    %state measurement
    X_observed = C*X + Wn*randn(size(C,1),1);
    Observed(:,i) = X_observed;
    
    %generating new inputs
    u = lqrController(K,X_observed,X_set,u_tilde); %noisy state
    score = score + calcReward(X_set,X_observed);
end
toc

%% plots
disp(['Observation is ',observation]);
disp(['Noise = ',num2str(noise),' Disturbance = ', num2str(disturbance)]);

plotStates(True,t, 'True values')
% plotStates(Observed,t, 'Measured values')
% plotFlight(True)