function plot_for_article(para, mc_results, cdf_results, model_names)
    % Generate figures for ISAC beam tracking paper
    % Inputs:
    %   mc_results: struct containing Monte Carlo simulation results
    %   cdf_results: struct containing CDF analysis results
    %   model_names: cell array of model names
    
    % Define baseline models
    baseline_models = {'perfect_state', 'random_bf'};

    % Setup figure properties for publication
    setupFigureProperties();
    
    % Define consistent styling
    styles = defineVisualStyles(model_names, baseline_models);
    
    %% Figure 1: Tracking Performance Analysis
    plotTrackingPerformance(mc_results, model_names, styles, para);
    
    %% Figure 2: Communication Performance Analysis
    plotCommunicationRate(mc_results, model_names, baseline_models, styles, para);
    
    %% Figure 3: CDF Analysis
    plotCDFAnalysis(cdf_results, model_names, styles);
    
    %% Figure 4: Beam Alignment Analysis
    plotBeamAlignment(mc_results, model_names, styles);
end

%% Helper Functions
function setupFigureProperties()
    % Set up figure properties for publication quality
    set(0, 'DefaultAxesFontName', 'Times New Roman');
    set(0, 'DefaultAxesFontSize', 14);
    set(0, 'DefaultTextFontName', 'Times New Roman');
end

function styles = defineVisualStyles(model_names, baseline_models)
    % Initialize styles struct
    styles = struct();
    
    % Basic color scheme for tracking models
    styles.colors = {
    [0, 0.4470, 0.7410],      % Blue
    [0.8500, 0.3250, 0.0980], % Orange
    [0.4940, 0.1840, 0.5560], % Purple
    [0.4660, 0.6740, 0.1880], % Green
    [0.9290, 0.6940, 0.1250]  % Yellow (新增，用于far-field模型)
};
    
    % Baseline colors
    styles.baseline_colors = {
        [0, 0, 0],        % Black for Perfect CSI
        [0.6, 0.6, 0.6]   % Gray for Random BF
    };
    
    % Line styles
    styles.lineStyles = {'-', '--', ':', '-.', '-.'};  % 用 '--.' 作为第五种线型
    
    % Markers
    styles.markers = {'o', 's', '^', 'v', 'diamond'};  % 增加一个'diamond'菱形标记
    styles.baseline_markers = {'diamond', 'square'};
    
    % Basic settings
    styles.lineWidth = 1.5;
    styles.markerSize = 8;
    styles.baseline_lineWidth = 1.0;  % 基准方法使用更细的线宽
    
    % Create style combinations for tracking models
    styles.modelStyles = cell(length(model_names), 1);
    for i = 1:length(model_names)
        styles.modelStyles{i} = struct(...
            'color', styles.colors{i}, ...
            'lineStyle', styles.lineStyles{i}, ...
            'marker', styles.markers{i} ...
        );
    end
    
    % Create style combinations for baseline models
    styles.baselineStyles = cell(length(baseline_models), 1);
    for i = 1:length(baseline_models)
        styles.baselineStyles{i} = struct(...
            'color', styles.baseline_colors{i}, ...
            'lineStyle', '-' ...  % 基准方法都使用实线
        );
    end
end


%% Fig1. 追踪精度对比图（距离+角度）
function plotTrackingPerformance(mc_results, model_names, styles, para)
    % Generate tracking performance comparison figure with log scale
    % Inputs:
    %   mc_results: struct containing Monte Carlo simulation results
    %   model_names: cell array of model names
    %   styles: struct containing visual styles
    %   para: struct containing simulation parameters
    
    % Create figure
    figure('Position', [100 100 900 300])
    
    % Calculate time points
    num_epochs = size(mc_results.(model_names{1}).rmse_position, 2);
    time_axis = (0:num_epochs-1) * para.epoch_interval * 0.05;
    
    % Position RMSE subplot
    subplot(1,2,1)
    for m = 1:length(model_names) -1
        % Calculate mean RMSE across all trials and vehicles
        mean_rmse = squeeze(mean(mc_results.(model_names{m}).rmse_position, [1 3]));
        
        % Plot with style
        semilogy(time_axis, mean_rmse, 'Color', styles.modelStyles{m}.color, ...
             'LineStyle', styles.modelStyles{m}.lineStyle, ...
             'LineWidth', styles.lineWidth, ...
             'Marker', styles.modelStyles{m}.marker, ...
             'MarkerSize', styles.markerSize, ...
             'MarkerIndices', 1:4:length(mean_rmse))
        hold on
    end
    
    % Customize plot
    xlabel('Time (s)')
    ylabel('RMSE of Distance (m)')
    % title('(a) Position Tracking Error')
    legend(strrep(model_names, '_', '-'), 'Location', 'best', 'Interpreter', 'none')
    grid on
    ax = gca;
    ax.GridAlpha = 0.15;
    ax.LineWidth = 1;
    ax.YMinorGrid = 'on';  % Show minor grid lines for log scale
    
    % Beam Alignment Error subplot
    subplot(1,2,2)
    for m = 1:length(model_names) -1
        % Calculate mean beam error across all trials and vehicles
        mean_beam = squeeze(mean(mc_results.(model_names{m}).beam_alignment_error, [1 3]));
        
        % Plot with style
        semilogy(time_axis, rad2deg(mean_beam), 'Color', styles.modelStyles{m}.color, ...
             'LineStyle', styles.modelStyles{m}.lineStyle, ...
             'LineWidth', styles.lineWidth, ...
             'Marker', styles.modelStyles{m}.marker, ...
             'MarkerSize', styles.markerSize, ...
             'MarkerIndices', 1:4:length(mean_beam))
        hold on
    end
    
    % Customize plot
    xlabel('Time (s)')
    ylabel('RMSE of angle (deg)')
    % title('(b) Beam Alignment Error')
    grid on
    ax = gca;
    ax.GridAlpha = 0.15;
    ax.LineWidth = 1;
    ax.YMinorGrid = 'on';  % Show minor grid lines for log scale
    
    % Adjust overall figure appearance
    set(gcf, 'Color', 'white');
    set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
    
end

%% 可达速率对比图，横轴时间
function plotCommunicationRate(mc_results, model_names, baseline_models, styles, para)
    % Create figure with square shape
    figure('Position', [100 100 600 450])
    
    % Calculate time points
    num_epochs = size(mc_results.(model_names{1}).achievable_rate, 2);
    time_axis = (0:num_epochs-1) * para.epoch_interval * 0.05;
    
    % Plot tracking models
    for m = 1:length(model_names) -1
        mean_rate = squeeze(mean(mc_results.(model_names{m}).achievable_rate, [1 3]));
        
        plot(time_axis, mean_rate, 'Color', styles.modelStyles{m}.color, ...
             'LineStyle', styles.modelStyles{m}.lineStyle, ...
             'LineWidth', styles.lineWidth, ...
             'Marker', styles.modelStyles{m}.marker, ...
             'MarkerSize', styles.markerSize, ...
             'MarkerIndices', 1:4:length(mean_rate))
        hold on
    end
    
    % Plot baseline models (only lines, no markers)
    for b = 1:length(baseline_models)
        mean_rate = squeeze(mean(mc_results.(baseline_models{b}).achievable_rate, [1 3]));
        
        plot(time_axis, mean_rate, 'Color', styles.baselineStyles{b}.color, ...
             'LineStyle', '-', ...
             'LineWidth', styles.baseline_lineWidth)
    end
    
    % Customize plot
    xlabel('Time (s)')
    ylabel('Achievable Rate (bps/Hz)')
    legend_names = [strrep(model_names, '_', '-'), 'Perfect CSI', 'Random BF'];
    legend(legend_names, 'Location', 'best', 'Interpreter', 'none')
    grid on
    
    % Adjust axes properties
    ax = gca;
    ax.GridAlpha = 0.15;
    ax.LineWidth = 1;
    
    % Set figure properties
    set(gcf, 'Color', 'white');
    set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
    
    % Make plot area more square-like
    set(gca, 'Position', [0.15 0.15 0.75 0.75])
end

%%
function plotCDFAnalysis(cdf_results, model_names, styles)
    % Generate CDF analysis figure for achievable rate and outage probability
    % Inputs:
    %   cdf_results: struct containing CDF analysis results
    %   model_names: cell array of model names
    %   styles: struct containing visual style definitions
    
    % Create figure
    figure('Position', [100 100 600 450])
    
    for m = 1:length(model_names)
        % Plot rate CDF with style
        plot(cdf_results.rate_cdf.(model_names{m}).rates, ...
             cdf_results.rate_cdf.(model_names{m}).probabilities, ...
             'Color', styles.modelStyles{m}.color, ...
             'LineStyle', styles.modelStyles{m}.lineStyle, ...
             'LineWidth', styles.lineWidth)
        hold on
    end
    
    % 添加far-field模型
    % plot(cdf_results.rate_cdf.imm_pda_farfield.rates, ...
    %      cdf_results.rate_cdf.imm_pda_farfield.probabilities, ...
    %      'Color', styles.colors{5}, ...
    %      'LineStyle', styles.lineStyles{1}, ...
    %      'LineWidth', styles.lineWidth)

    % Customize rate CDF plot
    xlabel('Achievable Rate (bps/Hz)')
    ylabel('CDF')
    % title('(a) Rate Distribution')
    grid on
    ax = gca;
    ax.GridAlpha = 0.15;
    ax.LineWidth = 1;
    ax.XLim = [0 max([cdf_results.rate_cdf.(model_names{1}).rates])];
    ax.YLim = [0 1];
    legend_names = [strrep(model_names, '_', '-')];
    legend(legend_names, 'Location', 'southeast', 'Interpreter', 'none')
    
    
    
    % Adjust overall figure appearance
    set(gcf, 'Color', 'white');
    set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
end

%%
function plotBeamAlignment(mc_results, model_names, styles)
    % Generate beam alignment analysis figure
    % figure('Position', [100 100 800 300]);
    % Plot beam alignment results
end