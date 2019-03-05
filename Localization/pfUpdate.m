function [samples, weight, mu, Sigma, predState, predMu, predSigma, zHat] = pfUpdate( ...
    samples, weight, numSamples, u, deltaT, M, z, Q, markerId)

% NOTE: The header is not set in stone.  You may change it if you like
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(markerId);
landmark_y = FIELDINFO.MARKER_Y_POS(markerId);

stateDim=3;
motionDim=3;
observationDim=2;


% ----------------------------------------------------------------
% Prediction step
% ----------------------------------------------------------------

% some stuff goes here

% Compute mean and variance of estimate. Not really needed for inference.
[predMu, predSigma] = meanAndVariance(samples, numSamples);


% ----------------------------------------------------------------
% Correction step
% ----------------------------------------------------------------

% more stuff goes here


