function [detected_targets] = cfar_detector(para, target_channels, tx_beamforming, rx_beamforming)
% CFAR_DETECTOR Constant False Alarm Rate detector for target detection
%
% Inputs:
%   para            - System parameters structure
%   target_channels - Target channel information structure array
%   tx_beamforming  - Transmit beamforming vector
%   rx_beamforming  - Receive beamforming vector
%
% Outputs:
%   detected_targets - Structure array of detected targets:
%                      r - Distance (m)
%                      theta - Angle (rad)
%                      speed - Target speed (m/s)
%                      yaw - Target heading (degrees)
%                      power - Signal power
%                      is_detected - Detection flag (boolean)
    
num_targets = length(target_channels);
detected_targets = struct('r', cell(1,num_targets), ...
                        'theta', cell(1,num_targets), ...
                        'speed', cell(1,num_targets), ...
                        'yaw', cell(1,num_targets), ...
                        'power', cell(1,num_targets), ...
                        'is_detected', cell(1,num_targets));

% Calculate CFAR detection threshold
% η = -2σ²ln(Pfa)
threshold = -2 * para.noise * log(para.Pfa);

% Process each target
for i = 1:num_targets
    % Calculate transmit beam gain
    kT = abs(target_channels(i).a_tx' * tx_beamforming)^2;
    % Calculate receive beam gain (using conjugate matching)
    kR = abs(rx_beamforming' * target_channels(i).a_rx)^2;
    
    % Calculate total signal power
    signal_power = para.Pt * para.G * abs(target_channels(i).beta)^2 * kT * kR;
    
    % CFAR detection
    is_detected = (signal_power >= threshold);
    
    % Store results
    detected_targets(i).r = target_channels(i).r;
    detected_targets(i).theta = target_channels(i).theta;
    detected_targets(i).speed = target_channels(i).speed;
    detected_targets(i).yaw = target_channels(i).yaw;
    detected_targets(i).power = signal_power;
    detected_targets(i).is_detected = is_detected;
end
end