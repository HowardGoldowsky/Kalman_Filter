classdef Track
    
    properties
        
        X_bel       % [M x I] best estimate state vector for iteration k
        X_pred      % [M x I] predicted state vector for iteration k-1
        R           % [N x N] sensor resolution covariance matrix
        A           % [M x M] prediction matrix; characterizes dynamics in prediction model
        B           % [M x J] control matrix; characterizes external forces
        U           % [J x 1] control vector; quantifies external forces
        Q           % [M x M] uncertainty due to the environment
        P           % [M x M] uncertainty between state vector components
        C           % [N x M] sensor matrix; bridges state vector and sensor

                    %   I = number of observations
                    %   J = number of external forces
                    %   M = number of states in state vector
                    %   N = number of distinct state sensors

    end
    
    methods
        
        function obj = Track(initObs, A, Q, B, U, P, R, C)
            
            obj.X_bel = initObs;  % initial observation is the first best estimate 
            obj.A = A;
            obj.R = R;
            obj.B = B;
            obj.U = U;
            obj.P = P;
            obj.Q = Q;
            obj.C = C;
            
        end
        
    end
end

