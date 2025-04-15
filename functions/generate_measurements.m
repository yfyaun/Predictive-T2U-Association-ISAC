function measurements = generate_measurements(para, detected_targets)
% GENERATE_MEASUREMENTS Generate measurements for detected targets with noise
%
% Inputs:
%   para            - System parameters structure
%   detected_targets - Structure array of detected targets from CFAR detector
%
% Outputs:
%   measurements    - Structure array of measurements for each detected target:
%                     r - Range measurement (m)
%                     theta - Angle measurement (rad)
%                     v_radial - Radial velocity measurement (m/s)
%                     snr - Signal-to-Noise Ratio (dB)

% Initialize measurements array
measurements = struct('r', {}, 'theta', {}, 'v_radial', {}, 'snr', {});

for i = 1:length(detected_targets)
    if detected_targets(i).is_detected
        % Calculate SNR in linear scale
        snr_linear = detected_targets(i).power / para.noise;
        snr_db = 10 * log10(snr_linear);
        
        % Calculate measurement variances based on CRLB
        sys_fac = 12^2;
        var_r = sys_fac * para.c^2 / (8 * pi^2 * para.band^2 * snr_linear);
        var_theta = sys_fac * 2 / (para.Nt * snr_linear);
        var_v = sys_fac * para.lambda^2 / (8 * pi^2 * para.time_transmit^2 * snr_linear);
        
        % Generate noisy measurements
        r = detected_targets(i).r + sqrt(var_r) * randn();
        theta = detected_targets(i).theta + sqrt(var_theta) * randn();
        
        % Calculate radial velocity with SNR-dependent noise
        true_v_radial = detected_targets(i).speed * ...
            cos(deg2rad(detected_targets(i).yaw) - detected_targets(i).theta);
        v_radial = true_v_radial + sqrt(var_v) * randn();
        
        % Store measurements
        measurements(end+1).r = r;
        measurements(end).theta = theta;
        measurements(end).v_radial = v_radial;
        measurements(end).snr = snr_db;
    end
end