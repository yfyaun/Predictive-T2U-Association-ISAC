function tracker = correctWithNN(tracker, measurements)
% Nearest Neighbor data association for ISAC tracking
%
% Inputs:
%   tracker      - IMM or EKF tracker object
%   measurements - Structure array containing measurements:
%                  r - Range (m)
%                  v_radial - Radial velocity (m/s)
%                  theta - Angle (rad)
%
% Outputs:
%   tracker      - Updated tracker object
    
    % Get predicted state and measurement info 
    if isa(tracker, 'trackingIMM')
        predictedState = tracker.State;
        measFcn = tracker.TrackingFilters{1}.MeasurementFcn;
    elseif isa(tracker, 'trackingEKF')
        predictedState = tracker.State;
        measFcn = tracker.MeasurementFcn;
    else
        error('Unsupported tracker type');
    end
    
    % Convert measurements struct array to matrix format
    num_meas = length(measurements);
    if num_meas > 0
        meas_matrix = zeros(3, num_meas);
        for i = 1:num_meas
            meas_matrix(:,i) = [measurements(i).r; 
                               measurements(i).v_radial;
                               measurements(i).theta];
        end
        
        % Calculate predicted measurement [range; v_radial; angle]
        predictedMeasurement = measFcn(predictedState);
        
        % Convert range & angle to x,y coordinates for predicted measurement
        pred_x = predictedMeasurement(1) * cos(predictedMeasurement(3));
        pred_y = predictedMeasurement(1) * sin(predictedMeasurement(3));
        
        % Find nearest measurement using Euclidean distance in x,y space
        distances = zeros(1, num_meas);
        for i = 1:num_meas
            % Convert range & angle to x,y for each measurement
            meas_x = meas_matrix(1,i) * cos(meas_matrix(3,i));
            meas_y = meas_matrix(1,i) * sin(meas_matrix(3,i));
            
            % Calculate Euclidean distance
            dx = meas_x - pred_x;
            dy = meas_y - pred_y;
            distances(i) = sqrt(dx^2 + dy^2);
        end
        
        [minDist, minIdx] = min(distances);
        
        % Update with nearest measurement if within validation gate

        gate_threshold = 20;
        if minDist < gate_threshold
            correct(tracker, meas_matrix(:,minIdx));
        end
    end
end