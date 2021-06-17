% Tests the Craziflie model using LQR and full state observation

%performs well. High penalty for height, roll and pitch angles
% ONLY WORKS FULLY OBSERVED!

%clc;clear all;close all;

noise = 0;
disturbance = 0;
observation = 'full'; %'full' or 'half'
%full = fully observed all 12 variables
%half = only measured position and angles (6 variables)

%% Equilibrium point
% equilibrium state
X_tilde = [0 0 0 0 0 0 0 0 0 0 0 0].'; %[x,y,z,vx,vy,vz,phi,theta,psi,omega_phi,omega_theta,omega_psi]

%equilibrium input
u_tilde = equilibriumInput(X_tilde);

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
%prediciton horizon N> 1
N =3;

%creating array with states history for plotting purposes
True = zeros(12,length(t)); %true (noiseless) results
Observed = zeros(12,length(t)); % noisy states

%covariance matrices
Wn = noise*eye(12); %measurement noise covariance matrix
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
    X_observed = X + Wn*randn(12,1);
    Observed(:,i) = X_observed;
    
    %generating new inputs
    u = MPController_full(N,X_observed,X_set,dt);
    score = score + calcReward(X_set,X_observed);
end
toc

%% plots
disp(['Observation is ',observation]);
disp(['Noise = ',num2str(noise),' Disturbance = ', num2str(disturbance)]);

plotStates(True,t, 'True values')
% plotStates(Observed,t, 'Measured values')
% plotFlight(True)