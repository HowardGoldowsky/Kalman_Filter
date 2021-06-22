% kalmanFiltINIT

function [deltaT, A, Q, B, U, P, R, C, X, truthdata, P0, V0, SENSOR_WORKING] = kalman_class_FiltINIT_extra_credit()

SENSOR_WORKING = false;

% time between steps (seconds)
deltaT = 1;                     
 
% Prediction matrix
A = [1 0 deltaT 0;...
     0 1 0 deltaT;...
     0 0 1 0;...
     0 0 0 1];
 
% Covariance of sensor readings
Q = [8 0 0 0;...
     0 8 0 0;...
     0 0 1 0;...
     0 0 0 1];
 
% Control matrix
B = [0 0 0 0;...
     0 0 0 0;...
     0 0 0 0;...
     0 0 0 0];

% Control vector
U = [0;...
     0;
     0;
     0];

% Initial covariance between state vector components
P = eye(size(A));

% Uncertainty due to the environment, what is used to create random walk in
% position
R = [0.25 0 0.5 0;...
     0 0.25 0 0.5;...
     0.5 0 1 0;...
     0 0.5 0 1];

% Sensor matrix; bridges state vector and sensor
C = eye(size(A));          % no transfer function

% initial position 2D case
P0 = [0;...                 
      0];                 

% initial velocity 2D case
V0 = [0;...                 
      0];                

X = [P0;...
     V0];    % initial state vector

mu_wind = 0;
var_wind = 1;
sigma_wind = sqrt(var_wind);

numSamples = 50;
[Px, Py, Vx, Vy] = ToyBalloonData(P0,V0,mu_wind,sigma_wind,deltaT,numSamples);
% Px(6)=10;

SENSOR_WORKING = ones(numSamples+1,1);
%SENSOR_WORKING = zeros(numSamples+1,1);
%SENSOR_WORKING = rand(numSamples+1,1);

truthdata = [Px; Py; Vx; Vy];
end % function