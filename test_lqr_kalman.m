% Tests the Craziflie model using LQR controller and kalman filter estimate
% for partially and/or noisy observations
% THIS MUST HAVE  A LITTLE BIT OF NOISE POWER


%fully observed (noisy or noiseless) performs well. Height has a steady state error.
%half observed has large steady state error in height. Noisy performs even
%worse

%needs agressive filter (large Q in lqr) to compensate

clc;clear all;close all;

noise = 0.1;
disturbance = 0.1;
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
    Q = diag([1 1 10000000 1 1 1 1000 1000 1 1 1 1]); %weights for [x,y,z,vx,vy,vz,phi,theta,psi,omega_phi,omega_theta,omega_psi]
    R = 1e6*eye(4); %weights for [tau_phi, tau_theta, tau_psi, thrust]
    
    K = lqr(A,B,Q,R);
else
    disp ('System is NOT controllable around equilibrium point')
end

%% Observability evaluation
Obs = obsv(A,C); %observability matrix
if rank(Obs) == length(X_tilde) %check observability condition
    disp('System is observable')
    %weights in this case are the disturbance and noise covariances
    Q = disturbance*eye(12);
    R = noise*eye(size(C,1));
    [Kf,~,~] = lqe(A,eye(12),C,100*Q,R);
   
else
    disp ('System is NOT observable')
end

%% Setting variables for simulation

% initial conditions
X = [0 0 0 0 0 0 0 0 0 0 0 0].'; %full state
u = [0 0 0 0].'; % input [tau_phi, tau_theta, tau_psi, thrust]
%changes in +phi -> changes in +Y
%changes in +theta -> changes in -X
%changes in psi -> no changes in coordinates (pan)

%auxliliary for intial conditions
X_filtered = [0 0 0 0 0 0 0 0 0 0 0 0].';
X_true = X; %true = initial state (already known)

%set state
X_set = [1 1 1 0 0 0 0 0 pi/6 0 0 0].';

%simulation times
dt = 0.01; %step size
t_end = 20;
t = 0:dt:t_end; %simulation times

%creating array with states history for plotting purposes
True = zeros(12,length(t)); %true (noiseless) results
Observed = zeros(size(C,1),length(t)); %partially observed (noisy) states
Filtered = zeros(12,length(t)); %full state estimation

%covariance matrices
Wn = noise*eye(size(C,1)); %measurement noise covariance matrix
Wd = disturbance*eye(12); %disturbance in state covariance matrix

%% loop to solve differential equation
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
    
    %estimating full and noiseless state with kalman filter
    X_filtered = kalmanFilter(X_observed, u, X_filtered, A, B, C, Kf, dt);
    Filtered(:,i) = X_filtered;
    
    %generating new inputs
    u = lqrController(K,X_filtered,X_set,u_tilde); %noisy state

end

%% plots
disp(['Observation is ',observation]);
disp(['Noise = ',num2str(noise),' Disturbance = ', num2str(disturbance)]);


plotStates(True,t, 'True values') % noiseless
plotFlight(True)
plotStates2(True,Observed, Filtered, t)
if isequal('full', observation)
    plotStates(Observed,t, 'Measured values') %should be almolst noisy
else
    plotStatesReduced(Observed,t, 'Measured values')
end
plotStates(Filtered,t, 'Filtered values') %should be almolst noiseless