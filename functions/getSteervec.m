function [a] = getSteervec(para, r, theta)
% GETSTEERVEC Calculate near-field array response vector
%
% Inputs:
%   para  - System parameters structure
%   r     - Distance to target (m)
%   theta - Angle to target (rad)
%
% Outputs:
%   a     - Array response vector for beamforming

n = ( 0 : (para.Nt-1) )' * para.d ; % Array element position vector
n = n-mean(n);

% Calculate distance adjustment for each antenna element
r = sqrt(r^2 + n.^2 - 2*r*n*cos(theta)) - r;

% Generate beamforming vector with phase adjustments
a = exp( -1i * 2 * pi * para.fc/para.c * r );

end