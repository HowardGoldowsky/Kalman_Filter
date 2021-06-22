% Kalman filter toy 

function [track] = kalmanFilter(track, truthData, SENSOR_WORKING)

% OUTPUT
%   track:    Track object 
%
% INPUT
%   track:    Track object
%   obs:      observations from sensor
%   SENSOR_WORKING
%   pSensorFail: probability of sensor failing
%
%   M = number of controls
%   N = number of states in state vector
%   K = number of distinct observation sensors

%   Track object properties and dimensions
    X_bel   = track.X_bel;   % [N x 1] (incoming best estimate) state vector for iteration k-1
    P       = track.P;       % [N x N] covariance between state vector components
    R       = track.R;       % [N x N] uncertainty due to the environment
    C       = track.C;       % [K x N] sensor matrix; bridges state vector and sensor   
    Q       = track.Q;       % [K x K] measurement covariance matrix   
    A       = track.A;       % [N x N] prediction matrix; characterizes dynamics in prediction model
    B       = track.B;       % [N x M] control matrix; characterizes external forces
    U       = track.U;       % [M x 1] control vector; quantifies external forces       
  
    % Prediction model
    X_pred = A*X_bel + B*U;         
    P = A*P*A' + R;
    
    % Incorporate observation
    if SENSOR_WORKING
        K = P*C' / (C*P*C' + Q);        % Kalman gain
        X_bel = X_pred + K*(truthData - C*X_pred);
        P = P - K*C*P;
    else
        X_bel = X_pred + truthData;     % do not combine obs
    end

    % Save changed track properties
    track.P = P;                    % covariance
    track.X_bel = X_bel;            % outgoing best estimate
    track.X_pred = X_pred;          % prediction
    
end % function