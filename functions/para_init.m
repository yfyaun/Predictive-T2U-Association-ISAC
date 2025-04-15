function [para] = para_init()
% PARA_INIT Initialize system parameters for ISAC beam tracking
%
% Outputs:
%   para - Structure containing all system parameters
%
% Notes:
%   This function defines physical constants, antenna configurations,
%   signal properties, and communication parameters

para.c = 3e8; % Speed of light in free space (m/s)
para.fc = 30e9; % Carrier frequency (Hz)
para.lambda = para.c / para.fc; % Wavelength (m)
para.band = 500e6; % Signal bandwidth (Hz)
para.fs = para.band; % Sampling rate (Hz)

para.Nt = 128; % Number of transmit antennas
para.Nr = 128; % Number of receive antennas
para.d = para.lambda / 2; % Antenna spacing (m)
para.D = para.d * (para.Nt-1); % Antenna aperture (m)

para.epoch_interval = 15; % Tracking epoch time = epoch_interval * trajectory_sample_rate (0.05s) 
                          %                     = 0.75s
para.time_transmit = 20000e-6; % Coherent processing interval (s)
para.symbol_rate = para.band; % Symbol rate (Hz)

para.K = 4; % Number of users
para.Pt = 10^(30/10); % Per antenna transmit power (mW)
para.noise = 10^(-57/10); % Noise power (mW)
para.G = para.time_transmit * para.symbol_rate; % Matched filter gain

end

