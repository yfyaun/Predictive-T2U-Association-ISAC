function [tracking_metrics, communication_metrics] = evaluate_performance(results)
% EVALUATE_PERFORMANCE Calculate tracking and communication performance metrics
%
% Inputs:
%   results - Structure containing simulation results for a single trial
%
% Outputs:
%   tracking_metrics     - Structure containing tracking performance metrics:
%                          rmse_position - Position error (m)
%                          rmse_velocity - Velocity error (m/s)
%                          beam_alignment_error - Angular error (rad)
%   communication_metrics - Structure containing communication performance metrics:
%                           achievable_rate - Spectral efficiency (bps/Hz)
%                           beam_gain - Channel gain

num_vehicles = length(results.vehicle);
num_epochs = length(results.vehicle(1).true_states.x);

% Initialize tracking metrics
tracking_metrics = struct(...
    'rmse_position', zeros(num_epochs, num_vehicles), ...
    'rmse_velocity', zeros(num_epochs, num_vehicles), ...
    'beam_alignment_error', zeros(num_epochs, num_vehicles));

% Initialize communication metrics
communication_metrics = struct(...
    'achievable_rate', zeros(num_epochs, num_vehicles), ...
    'beam_gain', zeros(num_epochs, num_vehicles));

% Calculate metrics for each vehicle and epoch
for v = 1:num_vehicles
    for epoch = 1:num_epochs
        if ~isempty(results.vehicle(v).predicted_states{epoch})
            % Get true and predicted states
            true_x = results.vehicle(v).true_states.x(epoch);
            true_y = results.vehicle(v).true_states.y(epoch);
            true_speed = results.vehicle(v).true_states.speed(epoch);
            
            pred_state = results.vehicle(v).predicted_states{epoch};
            pred_x = pred_state(1);
            pred_y = pred_state(3);
            pred_vx = pred_state(2);
            pred_vy = pred_state(4);
            pred_speed = sqrt(pred_vx^2 + pred_vy^2);
            
            % Position RMSE
            tracking_metrics.rmse_position(epoch, v) = ...
                sqrt((true_x - pred_x)^2 + (true_y - pred_y)^2);
            
            % Velocity RMSE
            tracking_metrics.rmse_velocity(epoch, v) = ...
                abs(true_speed - pred_speed);
            
            % Beam alignment error
            true_angle = atan2(true_y, true_x);
            pred_angle = atan2(pred_y, pred_x);
            tracking_metrics.beam_alignment_error(epoch, v) = ...
                abs(wrapToPi(true_angle - pred_angle));
        end
        
        % Communication metrics
        communication_metrics.achievable_rate(epoch, v) = ...
            results.vehicle(v).achievable_rates(epoch);
        
        if ~isempty(results.vehicle(v).channel_response{epoch})
            H = results.vehicle(v).channel_response{epoch};
            communication_metrics.beam_gain(epoch, v) = ...
                norm(H)^2;
        end
    end
end
end