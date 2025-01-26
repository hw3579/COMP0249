
matlab第一次写的代码
''' 
            global counter;
            % J = eye(2);
            % K = zeros(2);
            global J;
            global K;
            for o = 1 : numel(newLandmarks)
                % Uncomment these lines to store the index of the state
                % vector with the landmark ID
                offset = length(obj.x);
                landmarkIdx = offset + [1;2];
                obj.landmarkIDStateVectorMap = insert(obj.landmarkIDStateVectorMap, newLandmarks(o), offset);
                obj.x = [obj.x; event.data(:,o)];

                % disp(['xlen: ', num2str(length(obj.x))]);

                % Finish implementing activity 2 here

                J = [J; J(end-1:end,:)];
                K = [K; eye(2)];
                if length(J)~=4
                    J = [J,J(:,end-1:end)];
                    % K = [K,K];
                end
                obj.P = J * obj.P * J' + K * event.covariance * K';


                disp(['lenJ=x?: ', num2str(length(J)==length(obj.x))]);

            end
'''



在