function [mu, Sigma, predMu, predSigma, zhat, G, R, H, K ] = ukfUpdate( ...
    mu, Sigma, u, deltaT, M, z, Q, markerId)

% NOTE: The header is not set in stone.  You may change it if you like.
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(markerId);
landmark_y = FIELDINFO.MARKER_Y_POS(markerId);

stateDim=3;
motionDim=3;
observationDim=2;

% --------------------------------------------
% Setup UKF
% --------------------------------------------

% UKF params

% Augmented state

% Sigma points

% Weights


% --------------------------------------------
% Prediction step
% --------------------------------------------

% UKF prediction of mean and covariance


%--------------------------------------------------------------
% Correction step
%--------------------------------------------------------------

% UKF correction of mean and covariance
