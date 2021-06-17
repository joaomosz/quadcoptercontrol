% Tests the Craziflie model using PD controller tuned with linear plant
% ONLY WORKS FULLY OBSERVED!

%performs well for noiseless observation. Relative slow response...
%noisy

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

%% PD tuning using transfer function

s = tf('s'); %laplace variable
sI = s*eye(12);
G = C*inv(sI-A)*B+D; %MIMO transfer function

%proportional gains from pidTuner()
Kp_phi = 8.2859e-6;
Kp_theta = 8.3278e-6;
Kp_psi = 1.4631e-5;
Kp_z = 0.014;

%derivative gains from pidTuner()
Kd_phi = 1.4352e-5;
Kd_theta = 1.4424e-5;
Kd_psi = 2.5341e-5;
Kd_z = 0.024249;

%organazing gains in array
Kp = [Kp_phi Kp_theta Kp_psi Kp_z];
Kd = [Kd_phi Kd_theta Kd_psi Kd_z];


clear Kp_phi Kp_theta Kp_psi Kp_z Kd_phi Kd_theta Kd_psi Kd_z
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
X_set = [1 0 0 0 0 0 0 0 0 0 0 0].';
score = 0;
%simulation times
dt = 0.01; %step size
t_end = 40;
t = 0:dt:t_end; %simulation times

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
    u = pdController(Kp, Kd, X_set, X_observed, u_tilde);
    score = score + calcReward(X_set,X_observed);
end
toc

%% plots
disp(['Observation is ',observation]);
disp(['Noise = ',num2str(noise),' Disturbance = ', num2str(disturbance)]);


plotStates(True,t, 'True values')
% plotStates(Observed,t, 'Measured values')
% plotFlight(True)