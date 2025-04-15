% Monte Carlo simulation script
clear; clc; close all;

%% Parameters
num_trials = 1000;  % Number of Monte Carlo trials
model_names = {'imm_pda', 'imm_nn', 'cv_pda', 'cv_nn', 'imm_pda_farfield'};
baseline_models = {'perfect_state', 'random_bf'};

% Initialize results structure
mc_results = struct();
for m = 1:length(model_names)
   mc_results.(model_names{m}) = struct(...
       'rmse_position', [], ...
       'beam_alignment_error', [], ...
       'achievable_rate', []);
end

% For baseline models (only achievable rate)
for m = 1:length(baseline_models)
    mc_results.(baseline_models{m}) = struct('achievable_rate', []);
end

%% Add functions path
addpath('functions');
addpath('trajectory_data');

%% Monte Carlo Loop
for trial = 1:num_trials
   % Set random seed
   rng(trial, 'twister');
   
   % Run single trial
   para = para_init();
   para.epoch_interval = 15; % Tracking epoch time = epoch_interval * trajectory_sample_rate (0.05s) 
                             % = 0.75s
   para.clutter_params = struct(...
       'num', 3, ...
       'min_dist', 5, ...
       'max_dist', 8, ...
       'range_std', 3, ...
       'speed_std', 3);

   [tracking_metrics, communication_metrics] = single_trial_simulation(para);
   
   % Collect results for tracking models
   for m = 1:length(model_names)
       mc_results.(model_names{m}).rmse_position(trial,:,:) = ...
           tracking_metrics.(model_names{m}).rmse_position;
       mc_results.(model_names{m}).beam_alignment_error(trial,:,:) = ...
           tracking_metrics.(model_names{m}).beam_alignment_error;
       mc_results.(model_names{m}).achievable_rate(trial,:,:) = ...
           communication_metrics.(model_names{m}).achievable_rate;
   end

   % Collect results for baseline models (only achievable rate)
   for m = 1:length(baseline_models)
       mc_results.(baseline_models{m}).achievable_rate(trial,:,:) = ...
           communication_metrics.(baseline_models{m}).achievable_rate;
   end
   
   fprintf('Completed trial: %d/%d\n', trial, num_trials);
end

%% Calculate statistics
stats = struct();
for m = 1:length(model_names)
   % Mean values across trials
   stats.(model_names{m}).mean_rmse = mean(mc_results.(model_names{m}).rmse_position, 1);
   stats.(model_names{m}).mean_beam_error = mean(mc_results.(model_names{m}).beam_alignment_error, 1);
   stats.(model_names{m}).mean_rate = mean(mc_results.(model_names{m}).achievable_rate, 1);
   
   % Standard deviations
   stats.(model_names{m}).std_rmse = std(mc_results.(model_names{m}).rmse_position, 0, 1);
   stats.(model_names{m}).std_beam_error = std(mc_results.(model_names{m}).beam_alignment_error, 0, 1);
   stats.(model_names{m}).std_rate = std(mc_results.(model_names{m}).achievable_rate, 0, 1);
end

% Statistics for baseline models (only rate-related)
for m = 1:length(baseline_models)
    stats.(baseline_models{m}).mean_rate = mean(mc_results.(baseline_models{m}).achievable_rate, 1);
    stats.(baseline_models{m}).std_rate = std(mc_results.(baseline_models{m}).achievable_rate, 0, 1);
end

%% Save results
save('results/monte_carlo_results.mat', 'mc_results', 'stats');
