function mc_data = evaluate_monte_carlo(mc_results, model_names)
% EVALUATE_MONTE_CARLO Evaluate Monte Carlo simulation results for ISAC beam tracking
%
% Inputs:
%   mc_results  - Structure containing Monte Carlo simulation results
%   model_names - Cell array of model names to evaluate
%
% Outputs:
%   mc_data     - Structure containing:
%                 rate_cdf - Achievable rate CDF data
%                 outage_cdf - Outage probability CDF data
%                 performance_metrics - Key performance indicators

% Initialize output structure
mc_data = struct();

% Define rate thresholds for outage probability calculation
rate_thresholds = linspace(0, 22, 220); % From 0 to 22 bps/Hz with 220 points for smooth curve

% Process each model
for m = 1:length(model_names)
    current_model = model_names{m};
    
    % Collect all rate samples across Monte Carlo trials
    rates = mc_results.(current_model).achievable_rate;
    rate_samples = rates(:); % Convert to column vector
    
    % Calculate achievable rate CDF
    [sorted_rates, rate_cdf] = calculate_empirical_cdf(rate_samples);
    mc_data.rate_cdf.(current_model).rates = sorted_rates;
    mc_data.rate_cdf.(current_model).probabilities = rate_cdf;
    
    % Calculate outage probabilities for different rate thresholds
    outage_probs = zeros(size(rate_thresholds));
    for i = 1:length(rate_thresholds)
        outage_probs(i) = sum(rate_samples <= rate_thresholds(i)) / length(rate_samples);
    end
    mc_data.outage_cdf.(current_model).thresholds = rate_thresholds;
    mc_data.outage_cdf.(current_model).probabilities = outage_probs;
    
    % Calculate key performance metrics
    mc_data.performance_metrics.(current_model) = calculate_performance_metrics(...
        rate_samples, sorted_rates, rate_cdf);
end
end

function [sorted_values, cdf] = calculate_empirical_cdf(samples)
% Calculate empirical CDF from samples
% Inputs:
%   samples: Vector of sample values
% Outputs:
%   sorted_values: Sorted sample values
%   cdf: Corresponding CDF probabilities

sorted_values = sort(samples);
num_samples = length(sorted_values);
cdf = (1:num_samples)' / num_samples;
end

function metrics = calculate_performance_metrics(samples, sorted_samples, cdf)
% Calculate key performance metrics
% Inputs:
%   samples: Original sample values
%   sorted_samples: Sorted sample values
%   cdf: CDF probabilities
% Outputs:
%   metrics: Structure containing performance metrics

metrics = struct();

% Basic statistics
metrics.mean = mean(samples);
metrics.median = median(samples);
metrics.std = std(samples);
metrics.var = var(samples);

% Percentile statistics
metrics.percentile_90 = interp1(cdf, sorted_samples, 0.90);
metrics.percentile_95 = interp1(cdf, sorted_samples, 0.95);
metrics.percentile_99 = interp1(cdf, sorted_samples, 0.99);

% Calculate ergodic capacity
metrics.ergodic_capacity = mean(samples);

% Calculate common outage probabilities
common_thresholds = [1, 2, 3, 4]; % bps/Hz
metrics.outage_probabilities = zeros(size(common_thresholds));
for i = 1:length(common_thresholds)
    metrics.outage_probabilities(i) = sum(samples <= common_thresholds(i)) / length(samples);
end

% Calculate reliability metrics
metrics.reliability_90 = sum(samples >= metrics.percentile_90) / length(samples);
metrics.reliability_95 = sum(samples >= metrics.percentile_95) / length(samples);
metrics.reliability_99 = sum(samples >= metrics.percentile_99) / length(samples);
end