classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge summary of PlatformPredictionEdge
    %
    % This class stores the factor representing the process model which
    % transforms the state from timestep k to k+1
    %
    % The process model is as follows.
    %
    % Define the rotation vector
    %
    %   M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
    %
    % The new state is predicted from
    %
    %   x_(k+1) = x_(k) + M * [vx;vy;theta]
    %
    % Note in this case the measurement is actually the mean of the process
    % noise. It has a value of 0. The error vector is given by
    %
    % e(x,z) = inv(M) * (x_(k+1) - x_(k))
    %
    % Note this requires estimates from two vertices - x_(k) and x_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k) and slot 2 contains x_(k+1).
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function obj = PlatformPredictionEdge(dT)
            % PlatformPredictionEdge for PlatformPredictionEdge
            %
            % Syntax:
            %   obj = PlatformPredictionEdge(dT);
            %
            % Description:
            %   Creates an instance of the PlatformPredictionEdge object.
            %   This predicts the state from one timestep to the next. The
            %   length of the prediction interval is dT.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a PlatformPredictionEdge
            
            assert(dT >= 0);
            obj = obj@g2o.core.BaseBinaryEdge(3);
            obj.dT = dT;
        end
        
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of a platform.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the platform x_(k+1) given
            %   an estimate of the platform at time x_(k) and the control
            %   input u_(k+1), following the process model described in
            %   PlatformPredictionEdge.
        
            
            % Extract the current state x_k
            x_k = obj.edgeVertices{1}.x;  % x_k = [x; y; theta]
            
            % Extract control input u_k
            u_k = obj.z;  % u_k = [s_k; 0; psi_k]
            
            % Extract time step length
            dT = obj.dT;
            
            % Compute rotation matrix M_k
            theta_k = x_k(3);  % Extract orientation angle
            M_k = [cos(theta_k), -sin(theta_k), 0;
                sin(theta_k),  cos(theta_k), 0;
                0, 0, 1];
            
            % Predict next state x_(k+1)
            x_kp1 = x_k + dT * M_k * u_k;
            
            % Assign predicted state to the next vertex
            obj.edgeVertices{2}.x = x_kp1;
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error, which is the difference
            %   between the expected and actual state transition.
            
            
            % Extract the states
            x_k = obj.edgeVertices{1}.x;  % x_k = [x; y; theta]
            x_kp1 = obj.edgeVertices{2}.x; % x_(k+1)
            
            % Extract control input u_k
            u_k = obj.z; % Assume z is the control input u_k

            
            % Extract time step length
            dT = obj.dT;
            
            % Compute rotation matrix M_k
            theta_k = x_k(3);  % Extract orientation angle
            M_k = [cos(theta_k), -sin(theta_k), 0;
                sin(theta_k),  cos(theta_k), 0;
                0, 0, 1];
            
            % Compute predicted next state
            x_kp1_pred = x_k + dT * M_k * u_k;
            
            % Compute error
            obj.errorZ = M_k \ (x_kp1 - x_kp1_pred);
        end
        
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians for the edge.
            %
            % Syntax:
            %   obj.linearizeOplus();
            %
            % Description:
            %   Compute the Jacobians for the edge based on the current estimate.
            
            % Extract the current state x_k
            x_k = obj.edgeVertices{1}.x;
            
            % Compute rotation matrix M_k
            theta_k = x_k(3);
            M_k = [cos(theta_k), -sin(theta_k), 0;
                sin(theta_k),  cos(theta_k), 0;
                0, 0, 1];
            
            % Compute the inverse of M_k
            M_k_inv = inv(M_k);
            
            % Compute Jacobians
            obj.J{1} = -M_k_inv;  % Jacobian with respect to x_k
            obj.J{2} = M_k_inv;   % Jacobian with respect to x_(k+1)
        end
        
    end
end