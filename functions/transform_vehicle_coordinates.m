function [vehicle_x_coords, vehicle_y_coords] = transform_vehicle_coordinates(vehicle_x_coords, vehicle_y_coords, array_x, array_y)
% TRANSFORM_VEHICLE_COORDINATES Transform vehicle coordinates to array-centered coordinate system
%
% Inputs:
%   vehicle_x_coords - Cell array of vehicle x-coordinates
%   vehicle_y_coords - Cell array of vehicle y-coordinates
%   array_x          - X-coordinate of the antenna array in global coordinate system
%   array_y          - Y-coordinate of the antenna array in global coordinate system
%
% Outputs:
%   vehicle_x_coords - Cell array of transformed vehicle x-coordinates
%   vehicle_y_coords - Cell array of transformed vehicle y-coordinates
%
% Notes:
%   This function shifts the coordinates so that the antenna array is at the origin

    % Transform vehicle coordinates to array-centered coordinate system
    for i = 1:length(vehicle_x_coords)
        vehicle_x_coords{i} = vehicle_x_coords{i} - array_x;
        vehicle_y_coords{i} = vehicle_y_coords{i} - array_y;
    end
end
