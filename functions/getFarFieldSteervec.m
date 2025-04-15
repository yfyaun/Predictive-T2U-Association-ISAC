function [a] = getFarFieldSteervec(para, theta)
% GETFARFIELDSTEERVEC Calculate far-field array response vector
%
% Inputs:
%   para  - System parameters structure
%   theta - Angle to target (rad)
%
% Outputs:
%   a     - Far-field array response vector for beamforming
%
% Notes:
%   This creates a standard ULA steering vector for far-field approximation

% Generate antenna element index
n = (0:(para.Nt-1))';

% Calculate phase shifts for ULA
a = exp(-1i * 2 * pi * para.d * n * sin(theta) / para.lambda);

end