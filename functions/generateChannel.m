function [H_comm, target_channels] = generateChannel(para, user_state, clutter_vehicles)
% GENERATECHANNEL Generate communication channel and target channel responses
%
% Inputs:
%   para            - System parameters structure
%   user_state      - User vehicle state (position, speed, heading)
%   clutter_vehicles - Array of clutter vehicle states
%
% Outputs:
%   H_comm          - Communication channel response
%   target_channels - Structure array containing each target's channel information:
%                     a_tx   - Transmit steering vector
%                     a_rx   - Receive steering vector
%                     beta   - Path gain
%                     r      - Distance
%                     theta  - Angle
%                     speed  - Target speed
%                     yaw    - Target heading angle

% Combine user and clutter vehicle position information
num_clutter = length(clutter_vehicles);
all_positions = zeros(num_clutter + 1, 2);

% User position
all_positions(1,:) = [user_state.x, user_state.y];

% Clutter vehicle positions
for i = 1:num_clutter
    all_positions(i+1,:) = [clutter_vehicles(i).x, clutter_vehicles(i).y];
end

% Initialize target channel information structure array
target_channels = struct('a_tx', cell(1, num_clutter + 1), ...
                       'a_rx', cell(1, num_clutter + 1), ...
                       'beta', cell(1, num_clutter + 1), ...
                       'r', cell(1, num_clutter + 1), ...
                       'theta', cell(1, num_clutter + 1), ...
                       'speed', cell(1, num_clutter + 1), ...
                       'yaw', cell(1, num_clutter + 1));

% Assign user attributes
target_channels(1).speed = user_state.speed;
target_channels(1).yaw = user_state.yaw;

% Assign clutter vehicle attributes
for i = 1:num_clutter
    target_channels(i+1).speed = clutter_vehicles(i).speed;
    target_channels(i+1).yaw = clutter_vehicles(i).yaw;
end

% Calculate channel parameters for each target
for k = 1:size(all_positions,1)
    % Calculate distance and angle
    r = norm(all_positions(k,:));
    theta = atan2(all_positions(k,2), all_positions(k,1));
    
    % Generate transmit and receive steering vectors
    a_tx = getSteervec(para, r, theta);
    a_rx = getSteervec(para, r, theta);
    
    % Path gain calculation
    if k == 1  % User communication channel
        % Communication channel spherical wave path gain: β = sqrt(λ/4πr)exp(-j2πr/λ)
        beta = sqrt(para.lambda/(4*pi*r)) * exp(-1i*2*pi*r/para.lambda);
        % Construct communication channel response
        H_comm = beta * a_tx;
    else  % Radar echo channel
        % Radar echo channel two-way propagation loss: β = sqrt(λ/4πr²)exp(-j4πr/λ)
        % Add random reflection coefficient: β_reflection ~ CN(0,1)
        % Radar RCS reflection coefficient: β = λε/(4π)^(3/2)d^(2n)
        % Where ε is a complex Gaussian random variable, n is the path loss factor (default set to 2)
        % Passenger vehicle RCS modeling (10-20 dBsm)
        rcs_mean_db = 15; % Average RCS 15dBsm
        rcs_std_db = 1;   % Standard deviation 1dB 
        rcs = 10^((rcs_mean_db + rcs_std_db * randn(1))/10);
        epsilon = sqrt(rcs) * (randn(1) + 1i*randn(1))/sqrt(2);
        n = 1; % Path loss factor
        beta = para.lambda * epsilon / ((4*pi)^(3/2) * r^(2*n)) * exp(-1i*4*pi*r/para.lambda);
    end
    
    % Store target channel information
    target_channels(k).a_tx = a_tx;
    target_channels(k).a_rx = a_rx;
    target_channels(k).beta = beta;
    target_channels(k).r = r;
    target_channels(k).theta = theta;
end
end