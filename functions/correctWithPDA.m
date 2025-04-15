function tracker = correctWithPDA(tracker, measurements)
% CORRECTWITHPDA Update IMM/EKF tracker with Probabilistic Data Association
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
    
% Get predicted state and measurement information
if isa(tracker, 'trackingIMM')
    predictedState = tracker.State;
    measFcn = tracker.TrackingFilters{1}.MeasurementFcn;
    measJacFcn = tracker.TrackingFilters{1}.MeasurementJacobianFcn;
    measNoise = tracker.MeasurementNoise;
elseif isa(tracker, 'trackingEKF')
    predictedState = tracker.State;
    measFcn = tracker.MeasurementFcn; 
    measJacFcn = tracker.MeasurementJacobianFcn;
    measNoise = tracker.MeasurementNoise;
else
    error('Unsupported tracker type');
end

% Convert measurement structures to matrix form [range; radial_velocity; angle]
n_meas = length(measurements);
if n_meas == 0
    return;  % Nothing to update if no measurements
end

meas_matrix = zeros(3, n_meas);
for i = 1:n_meas
    meas_matrix(:,i) = [measurements(i).r; 
                       measurements(i).v_radial; 
                       measurements(i).theta];
end

% Calculate predicted measurement and covariance
predictedMeasurement = measFcn(predictedState);
measJac = measJacFcn(predictedState);
stateCov = tracker.StateCovariance;
predictedCov = measJac * stateCov * measJac';

% PDA parameters
Pd = 0.99;  % Detection probability 
lambda = 0.04; % Clutter density
S = predictedCov + measNoise;
[m,n] = size(meas_matrix);

if cond(S) > 1e12
    warning('Innovation covariance matrix poorly conditioned');
    % Apply regularization
    S = S + 1e-6 * eye(size(S));
end

% Compute log-likelihoods
log_likelihoods = zeros(1, n_meas);
for i = 1:n_meas
    innovation = meas_matrix(:,i) - predictedMeasurement;
    
    % Use log-sum-exp trick
    log_likelihoods(i) = -0.5 * (m*log(2*pi) + log(det(S)) + ...
        innovation' * (S\innovation)) + log(Pd) - log(lambda);
end

% Convert to probability domain using log-sum-exp trick
max_log_likelihood = max(log_likelihoods);
likelihoods = exp(log_likelihoods - max_log_likelihood);
total_likelihood = sum(likelihoods);

% beta_0 = lambda * (2*pi)^(3/2) * sqrt(det(S)) * (1-Pd);
beta = likelihoods / total_likelihood;

% Combine measurements
combinedZ = zeros(3,1);
for i = 1:n_meas
    combinedZ = combinedZ + beta(i) * meas_matrix(:,i);
end

% Calculate combined measurement covariance
combinedR = zeros(3,3);
% Measurement noise part
for i = 1:n_meas
    combinedR = combinedR + beta(i) * measNoise;
end

originalNoise = tracker.MeasurementNoise;
tracker.MeasurementNoise = combinedR;
correct(tracker, combinedZ);
tracker.MeasurementNoise = originalNoise;

end