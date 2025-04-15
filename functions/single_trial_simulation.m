function [tracking_metrics, communication_metrics] = single_trial_simulation(para)
% SINGLE_TRIAL_SIMULATION Run a single trial of ISAC beam tracking simulation
%
% Inputs:
%   para - Structure containing system parameters:
%          epoch_interval - Interval between execution epochs
%          clutter_params - Clutter vehicle generation parameters
%          Additional parameters are set within the function
%
% Outputs:
%   tracking_metrics     - Structure containing tracking performance metrics for each model
%   communication_metrics - Structure containing communication performance metrics for each model
%
% Notes:
%   This function simulates multiple tracking models and baseline approaches
%   and evaluates their performance in an ISAC beam tracking scenario

   % Initialize parameters
   
   
   para.cv_trans_prob = 0.9;
   para.ct_trans_prob = 0.9; 
   para.sigma_theta = 5e-3;      % Angle measurement std (rad)
   para.sigma_range = 2e-3;       % Range measurement std (m)
   para.sigma_speed = 1e-2;       % Velocity measurement std (m/s)
   para.Pfa = 1e-6;
   para.rsu_pos = [50,30];
   

   % Load and process trajectory data
   [vehicles_x, vehicles_y, vehicles_speeds, vehicles_yaws] = extract_vehicle_data('trajectory_data/vehicle_trajectory.csv');
   % Remove vehicle1 by taking all vehicles except the first one
   vehicles_x = vehicles_x(2:end);
   vehicles_y = vehicles_y(2:end);
   vehicles_speeds = vehicles_speeds(2:end);
   vehicles_yaws = vehicles_yaws(2:end);
   [vehicles_x, vehicles_y] = transform_vehicle_coordinates(vehicles_x, vehicles_y, para.rsu_pos(1), para.rsu_pos(2));
   
   num_epochs = length(vehicles_x{1});
   num_vehicles = length(vehicles_x);
   execution_epochs = 1:para.epoch_interval:num_epochs;
   num_executions = length(execution_epochs);

   % Initialize results structure
   model_names = {'imm_pda', 'imm_nn', 'cv_pda', 'cv_nn', 'imm_pda_farfield'};

   baseline_models = {'perfect_state', 'random_bf'};  % baseline方法
   % 添加随机波束成形的参数
   para.random_bf = struct(...
       'r_min', 5, ...    % 最小距离
       'r_max', 100, ...   % 最大距离
       'theta_min', -pi/2, ... % 最小角度
       'theta_max', pi/2   ... % 最大角度
   );

   results = struct();
   for m = 1:length(model_names)
       results.(model_names{m}) = struct('vehicle', []);
       for v = 1:num_vehicles
           results.(model_names{m}).vehicle(v).true_states = struct(...
               'x', zeros(1, num_executions), ...
               'y', zeros(1, num_executions), ...
               'speed', zeros(1, num_executions), ...
               'yaw', zeros(1, num_executions));
           
           results.(model_names{m}).vehicle(v).clutter_vehicles = cell(1, num_executions);
           results.(model_names{m}).vehicle(v).channel_response = cell(1, num_executions);
           results.(model_names{m}).vehicle(v).measurements = cell(1, num_executions);
           results.(model_names{m}).vehicle(v).state_estimates = cell(1, num_executions);
           results.(model_names{m}).vehicle(v).predicted_states = cell(1, num_executions);
           results.(model_names{m}).vehicle(v).beam_patterns = cell(1, num_executions);
           results.(model_names{m}).vehicle(v).achievable_rates = zeros(1, num_executions);
       end
   end

   % baseline models初始化
   for m = 1:length(baseline_models)
       results.(baseline_models{m}) = struct('vehicle', []);
       for v = 1:num_vehicles
           results.(baseline_models{m}).vehicle(v).true_states = struct(...
               'x', zeros(1, num_executions), ...
               'y', zeros(1, num_executions), ...
               'speed', zeros(1, num_executions), ...
               'yaw', zeros(1, num_executions));
           results.(baseline_models{m}).vehicle(v).channel_response = cell(1, num_executions);
           results.(baseline_models{m}).vehicle(v).predicted_states = cell(1, num_executions);
           results.(baseline_models{m}).vehicle(v).achievable_rates = zeros(1, num_executions);
       end
   end

   % Initialize trackers
   trackers = struct();
   for m = 1:length(model_names)
       trackers.(model_names{m}) = cell(1, num_vehicles);
   end

   achieved_rates = struct();
   for m = 1:length(model_names)
       achieved_rates.(model_names{m}) = zeros(num_executions, num_vehicles);
   end

   %% Main simulation loop
   for exec_idx = 1:num_executions
       epoch = execution_epochs(exec_idx);
       
       for v = 1:num_vehicles
           % Update true states
           current_state = struct(...
               'x', vehicles_x{v}(epoch), ...
               'y', vehicles_y{v}(epoch), ...
               'speed', vehicles_speeds{v}(epoch), ...
               'yaw', vehicles_yaws{v}(epoch));
           
           for m = 1:length(model_names)
               results.(model_names{m}).vehicle(v).true_states.x(exec_idx) = current_state.x;
               results.(model_names{m}).vehicle(v).true_states.y(exec_idx) = current_state.y;
               results.(model_names{m}).vehicle(v).true_states.speed(exec_idx) = current_state.speed;
               results.(model_names{m}).vehicle(v).true_states.yaw(exec_idx) = current_state.yaw;
           end
           
           % Initialize trackers at first epoch
           if exec_idx == 1
               initial_measurement = struct(...
                   'r', sqrt(current_state.x^2 + current_state.y^2), ...
                   'theta', atan2(current_state.y, current_state.x), ...
                   'v_radial', current_state.speed * cos(deg2rad(current_state.yaw) - atan2(current_state.y, current_state.x)));
               
               [imm_tracker1, cv_tracker1, ~] = createIMM(initial_measurement, para);
               [imm_tracker2, cv_tracker2, ~] = createIMM(initial_measurement, para);
               [imm_tracker_farfield, ~, ~] = createIMM(initial_measurement, para);
               trackers.imm_pda{v} = imm_tracker1;
               trackers.imm_nn{v} = imm_tracker2;
               trackers.cv_pda{v} = cv_tracker1;  
               trackers.cv_nn{v} = cv_tracker2;
               trackers.imm_pda_farfield{v} = imm_tracker_farfield;
               continue;
           end
           
           % Process each model
           for m = 1:length(model_names)
               current_tracker = trackers.(model_names{m}){v};
               
               % Prediction and beamforming
               deltaT = 0.05 * para.epoch_interval;
               predicted_state = predict(current_tracker, deltaT);
               results.(model_names{m}).vehicle(v).predicted_states{exec_idx} = predicted_state;
               
               predicted_r = sqrt(predicted_state(1)^2 + predicted_state(3)^2);
               predicted_theta = atan2(predicted_state(3), predicted_state(1));

               % 在波束赋形部分添加条件判断
               if strcmp(model_names{m}, 'imm_pda_farfield')
                   tx_beamforming = 1/sqrt(para.Nt) * getFarFieldSteervec(para, predicted_theta);
                   rx_beamforming = 1/sqrt(para.Nr) * getFarFieldSteervec(para, predicted_theta);
               else
                   tx_beamforming = 1/sqrt(para.Nt) * getSteervec(para, predicted_r, predicted_theta);
                   rx_beamforming = 1/sqrt(para.Nr) * getSteervec(para, predicted_r, predicted_theta);
               end
               
               % Generate clutter and channel
               clutter_vehicles = generate_clutter_vehicles(...
                   current_state.x, current_state.y, ...
                   current_state.speed, current_state.yaw, ...
                   para.clutter_params);
               
               [H_comm, target_channels] = generateChannel(para, current_state, clutter_vehicles);
               
               results.(model_names{m}).vehicle(v).clutter_vehicles{exec_idx} = clutter_vehicles;
               results.(model_names{m}).vehicle(v).channel_response{exec_idx} = H_comm;
               
               % CFAR detection and measurement generation
               detected_targets = cfar_detector(para, target_channels, tx_beamforming, rx_beamforming);
               measurements = generate_measurements(para, detected_targets);
               results.(model_names{m}).vehicle(v).measurements{exec_idx} = measurements;
               
               % Tracker update
               if ~isempty(measurements)
                   if contains(model_names{m}, 'pda')
                       correctWithPDA(current_tracker, measurements);
                   else
                       correctWithNN(current_tracker, measurements);
                   end
               end

               % Calculate performance metrics
               SNR = para.Pt * abs(H_comm' * tx_beamforming)^2 / para.noise;
               achieved_rates.(model_names{m})(exec_idx, v) = log2(1 + SNR);
               results.(model_names{m}).vehicle(v).achievable_rates(exec_idx) = ...
                   achieved_rates.(model_names{m})(exec_idx, v);
           end

           % 1. Perfect CSI
           true_r = sqrt(current_state.x^2 + current_state.y^2);
           true_theta = atan2(current_state.y, current_state.x);
           tx_beamforming_perfect = 1/sqrt(para.Nt) * getSteervec(para, true_r, true_theta);
           [H_comm_perfect, ~] = generateChannel(para, current_state, clutter_vehicles);
           SNR_perfect = para.Pt * abs(H_comm_perfect' * tx_beamforming_perfect)^2 / para.noise;
           results.perfect_state.vehicle(v).achievable_rates(exec_idx) = log2(1 + SNR_perfect);
           
           % 2. Random Beamforming
           random_r = para.random_bf.r_min + (para.random_bf.r_max - para.random_bf.r_min)*rand();
           random_theta = para.random_bf.theta_min + (para.random_bf.theta_max - para.random_bf.theta_min)*rand();
           tx_beamforming_random = 1/sqrt(para.Nt) * getSteervec(para, random_r, random_theta);
           [H_comm_random, ~] = generateChannel(para, current_state, clutter_vehicles);
           SNR_random = para.Pt * abs(H_comm_random' * tx_beamforming_random)^2 / para.noise;
           results.random_bf.vehicle(v).achievable_rates(exec_idx) = log2(1 + SNR_random);

       end
   end

   % Calculate performance metrics
   tracking_metrics = struct();
   communication_metrics = struct();
   for m = 1:length(model_names)
       [tracking_metrics.(model_names{m}), communication_metrics.(model_names{m})] = ...
           evaluate_performance(results.(model_names{m}));
   end

   % 对baseline models只评估通信性能
   for m = 1:length(baseline_models)
       % 不需要tracking metrics
       tracking_metrics.(baseline_models{m}) = [];
        
       % 只记录通信性能
       communication_metrics.(baseline_models{m}).achievable_rate = zeros(num_executions, num_vehicles);
       for v = 1:num_vehicles
           communication_metrics.(baseline_models{m}).achievable_rate(:,v) = ...
               results.(baseline_models{m}).vehicle(v).achievable_rates';
       end
       % beam_gain字段保持为空或设为[]
       communication_metrics.(baseline_models{m}).beam_gain = [];
   end
end