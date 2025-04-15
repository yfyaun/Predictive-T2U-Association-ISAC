function [vehicles_x_coords, vehicles_y_coords, vehicles_speeds, vehicles_yaws, timestamps] = extract_vehicle_data(filename)
% EXTRACT_VEHICLE_DATA Extract vehicle trajectory data from CSV file
%
% Inputs:
%   filename - Path to the CSV file containing vehicle trajectory data
%
% Outputs:
%   vehicles_x_coords - Cell array of x-coordinates for each vehicle
%   vehicles_y_coords - Cell array of y-coordinates for each vehicle
%   vehicles_speeds   - Cell array of speeds for each vehicle
%   vehicles_yaws     - Cell array of heading angles for each vehicle
%   timestamps        - Vector of timestamps for the trajectory

data = readtable(filename);
vehicle_ids = unique(data.Vehicle_ID);

% Use timestamps from the last vehicle
last_vehicle_data = data(data.Vehicle_ID == max(vehicle_ids), :);
timestamps = last_vehicle_data.Timestamp;

num_vehicles = length(vehicle_ids);
vehicles_x_coords = cell(num_vehicles, 1);
vehicles_y_coords = cell(num_vehicles, 1);
vehicles_speeds = cell(num_vehicles, 1);
vehicles_yaws = cell(num_vehicles, 1);

for i = 1:num_vehicles
    vehicle_data = data(data.Vehicle_ID == vehicle_ids(i), :);
    [~, time_idx] = ismember(timestamps, vehicle_data.Timestamp);
    
    vehicles_x_coords{i} = vehicle_data.X(time_idx);
    vehicles_y_coords{i} = vehicle_data.Y(time_idx);
    vehicles_speeds{i} = vehicle_data.Speed(time_idx);
    vehicles_yaws{i} = vehicle_data.Yaw(time_idx);
end
end