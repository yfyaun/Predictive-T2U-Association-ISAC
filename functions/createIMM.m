function [imm, ekf_CV, ekf_CT] = createIMM(initialMeasurement, para)
% CREATEIMM Create Interacting Multiple Model (IMM) filter with CV and CT models
%
% Inputs:
%   initialMeasurement - Structure containing initial measurements:
%                        r - Range (m)
%                        theta - Angle (rad)
%                        v_radial - Radial velocity (m/s)
%   para              - System parameters structure
%
% Outputs:
%   imm     - IMM filter object combining CV and CT models
%   ekf_CV  - Constant Velocity EKF filter
%   ekf_CT  - Constant Turn Rate EKF filter

% Define measurement noise
measurementNoise = diag([para.sigma_range^2, para.sigma_speed^2, para.sigma_theta^2]);

% Initial state
r_init = initialMeasurement.r;
theta_init = initialMeasurement.theta;
x_init = r_init * cos(theta_init);
y_init = r_init * sin(theta_init);
vx_init = initialMeasurement.v_radial * cos(theta_init);
vy_init = initialMeasurement.v_radial * sin(theta_init);

% Initial state covariance
stateCovarianceCV = diag([10, 4, 10, 4]); % CV: x, vx, y, vy
stateCovarianceCT = diag([10, 4, 10, 4, 2]); % CT: x, vx, y, vy, omega

% Set time interval and process noise parameters
dt = 0.05 * para.epoch_interval;  % Time step (s)
sigma_ax = 2^2;    % Acceleration variance x-direction (m/s²)²
sigma_ay = 2^2;    % Acceleration variance y-direction (m/s²)²
sigma_aw = 3^2;    % Angular acceleration variance (rad/s²)²

% For CV model - Continuous time process noise discretization
processNoiseCV = [
    dt^4/4*sigma_ax,  0,                0,                0;
    0,                dt^2*sigma_ax,    0,                0;
    0,                0,                dt^4/4*sigma_ay,  0;
    0,                0,                0,                dt^2*sigma_ay;
];

% For CT model - Extended for turn rate 
processNoiseCT = [
    dt^4/4*sigma_ax,  0,                0,                0,                0;
    0,                dt^2*sigma_ax,    0,                0,                0;
    0,                0,                dt^4/4*sigma_ay,  0,                0;
    0,                0,                0,               dt^2*sigma_ay,     0;
    0,                0,                0,                0,                dt^2*sigma_aw;
];

% Create Constant Velocity (CV) EKF
ekf_cv = trackingEKF(@constvel, @measurementFcn, [x_init; vx_init; y_init; vy_init], ...
    'StateCovariance', stateCovarianceCV, ...
    'ProcessNoise', processNoiseCV, ...
    'StateTransitionJacobianFcn', @constveljac, ...
    'MeasurementJacobianFcn', @measurementJacobianCV, ...
    'MeasurementNoise', measurementNoise);

% Create Constant Turn (CT) EKF
ekf_ct = trackingEKF(@constturn, @measurementFcn, [x_init; vx_init; y_init; vy_init; 0], ...
    'StateCovariance', stateCovarianceCT, ...
    'ProcessNoise', processNoiseCT, ...
    'StateTransitionJacobianFcn', @constturnjac, ...
    'MeasurementJacobianFcn', @measurementJacobianCT, ...
    'MeasurementNoise', measurementNoise);

% Define transition probability matrix
transitionProbabilities = [para.cv_trans_prob, 1-para.cv_trans_prob;  
                           1-para.ct_trans_prob, para.ct_trans_prob];

% Create IMM
imm = trackingIMM({ekf_cv, ekf_ct}, @switchimm, transitionProbabilities);

% Create separate CV and CT EKFs for comparison
ekf_CV = trackingEKF(@constvel, @measurementFcn, [x_init; vx_init; y_init; vy_init], ...
    'StateCovariance', stateCovarianceCV, ...
    'ProcessNoise', processNoiseCV, ...
    'StateTransitionJacobianFcn', @constveljac, ...
    'MeasurementJacobianFcn', @measurementJacobianCV, ...
    'MeasurementNoise', measurementNoise);
ekf_CT = trackingEKF(@constturn, @measurementFcn, [x_init; vx_init; y_init; vy_init; 0], ...
    'StateCovariance', stateCovarianceCT, ...
    'ProcessNoise', processNoiseCT, ...
    'StateTransitionJacobianFcn', @constturnjac, ...
    'MeasurementJacobianFcn', @measurementJacobianCT, ...
    'MeasurementNoise', measurementNoise);
end

% Measurement function
function z = measurementFcn(state)
    % State [x; vx; y; vy]
    x = state(1);
    vx = state(2);
    y = state(3);
    vy = state(4);
    % Range
    r = sqrt(x^2 + y^2);
    % Radial velocity
    rv = (x * vx + y * vy) / r;
    % Angle
    theta = atan2(y, x);
    z = [r; rv; theta];
end

function H = measurementJacobianCV(state)
    x = state(1);
    vx = state(2);
    y = state(3);
    vy = state(4);
    r = sqrt(x^2 + y^2);
    r2 = r^2;
    % Jacobian matrix
    H = [
        x/r,       0, y/r,       0;        % Range partial derivatives
        (vx*y - vy*x)/r2, x/r, (vy*x - vx*y)/r2, y/r; % Radial velocity partial derivatives
        -y/r2,     0, x/r2,      0         % Angle partial derivatives
    ];
end

function H = measurementJacobianCA(state)
    x = state(1);
    vx = state(2);
    ax = state(3);
    y = state(4);
    vy = state(5);
    ay = state(6);
    r = sqrt(x^2 + y^2);
    r2 = r^2;
    % Jacobian matrix
    H = [
        x/r,       0,    0, y/r,       0,    0;       % Range partial derivatives
        (vx*y - vy*x)/r2, x/r, ax/r, (vy*x - vx*y)/r2, y/r, ay/r; % Radial velocity partial derivatives
        -y/r2,     0,    0, x/r2,      0,    0        % Angle partial derivatives
    ];
end

function H = measurementJacobianCT(state)
    x = state(1);
    vx = state(2);
    y = state(3);
    vy = state(4);
    omega = state(5); % Angular velocity not directly involved in measurement model but exists in state space
    r = sqrt(x^2 + y^2);
    r2 = r^2;
    % Jacobian matrix
    H = [
        x/r,       0, y/r,       0,    0;       % Range partial derivatives
        (vx*y - vy*x)/r2, x/r, (vy*x - vx*y)/r2, y/r, 0; % Radial velocity partial derivatives
        -y/r2,     0, x/r2,      0,    0        % Angle partial derivatives
    ];
end