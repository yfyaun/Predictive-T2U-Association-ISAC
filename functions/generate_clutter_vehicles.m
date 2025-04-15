function clutter_vehicles = generate_clutter_vehicles(x, y, speed, yaw, clutter_params)
% GENERATE_CLUTTER_VEHICLES Generate clutter vehicles for radar simulation
%
% Inputs:
%   x, y            - User vehicle position coordinates
%   speed           - User vehicle speed
%   yaw             - User vehicle heading angle (degrees)
%   clutter_params  - Clutter parameters structure:
%                     num       - Number of clutter vehicles
%                     min_dist  - Minimum distance constraint
%                     max_dist  - Maximum distance constraint
%                     range_std - Position standard deviation
%                     speed_std - Speed standard deviation
%
% Outputs:
%   clutter_vehicles - Structure array of clutter vehicle states:
%                      x     - x-coordinate
%                      y     - y-coordinate
%                      speed - velocity magnitude
%                      yaw   - heading angle (degrees)

n_clutter = clutter_params.num;
min_dist = clutter_params.min_dist;
max_dist = clutter_params.max_dist;

% Initialize clutter vehicle state structure array
clutter_vehicles = struct('x', cell(1, n_clutter), ...
                        'y', cell(1, n_clutter), ...
                        'speed', cell(1, n_clutter), ...
                        'yaw', cell(1, n_clutter));

for i = 1:n_clutter
    while true
        % Generate clutter vehicle position in x-y coordinates
        clutter_x = x + clutter_params.range_std * randn();
        clutter_y = y + clutter_params.range_std * randn();
        
        % Check distance to user vehicle
        dist_to_real = sqrt((clutter_x - x)^2 + (clutter_y - y)^2);
        
        % Check distances to other clutter vehicles
        dist_ok = true;
        for j = 1:i-1
            if j > 0
                other_x = clutter_vehicles(j).x;
                other_y = clutter_vehicles(j).y;
                dist = sqrt((clutter_x - other_x)^2 + (clutter_y - other_y)^2);
                if dist < min_dist
                    dist_ok = false;
                    break;
                end
            end
        end
        
        % If distance constraints are satisfied, save this clutter vehicle state
        if dist_to_real >= min_dist && dist_to_real <= max_dist && dist_ok
            % Use same heading as user vehicle
            clutter_yaw = yaw;
            % Add random perturbation to speed
            clutter_speed = speed + clutter_params.speed_std * randn();
            
            % Save clutter vehicle state
            clutter_vehicles(i).x = clutter_x;
            clutter_vehicles(i).y = clutter_y;
            clutter_vehicles(i).speed = clutter_speed;
            clutter_vehicles(i).yaw = clutter_yaw;
            break;
        end
    end
end
end