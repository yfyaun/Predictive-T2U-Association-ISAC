% Monte Carlo visualization script
clear; clc; close all;

%% Parameters
model_names = {'imm_pda', 'imm_nn', 'cv_pda', 'cv_nn', 'imm_pda_farfield'};
baseline_models = {'perfect_state', 'random_bf'};
all_models = [model_names, baseline_models]; 

%% Add functions path
addpath('functions');
addpath('trajectory_data');

%% Load results
load('results/monte_carlo_results.mat');  % Load mc_results and stats

%% Load system parameters
para = para_init();

%% Generate CDF results
cdf_results = evaluate_monte_carlo(mc_results, model_names);

%% Visualization
plot_for_article(para, mc_results, cdf_results, model_names);

% Print completion message
fprintf('\nVisualization completed. Figures have been generated.\n');