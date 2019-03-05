function [mu, Sigma, predMu, predSigma] = ekfUpdate( ...
    muIn, SigmaIn, motionCommand, alphas, G, z, R, markerId)
% input
%%%%%%%%%%%%%%%%
% mu: mean of last state
% sigma: covariance of last state
% u: the control input
% deltaT: the period of update
% G : the jacobien of the linearized motion model to state
% V : the jacobien of the linearized motion model to control input 
% M : the covariance of the noise for control
% H : the jacobien of the linearized senser model
% Z : measurement at this time step
% Q : measurement noise covariance
% markerID : the ID of the marker

% output
%%%%%%%%%%%%%%%%
% mu : the output mean 
% sigma : the output variance
% predMu : the mu after prediction step
% preSigma : the sigma after prediction step
% zhat : the estimate output 
% K : kalman gain


% NOTE: The header is not set in stone.  You may change it if you like.

% Get the global position of the present marker 
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(markerId);
landmark_y = FIELDINFO.MARKER_Y_POS(markerId);

stateDim=3;
motionDim=3;
observationDim=2;



rot1 = motionCommand(1);
trans = motionCommand(2);
rot2 = motionCommand(3);
sigma1_temp = alphas(1)*rot1^2 + alphas(2)*trans^2;
sigma2_temp = alphas(3)*trans^2 + alphas(4)*rot1^2 + alphas(4)*rot2^2;
sigma3_temp = alphas(1)*rot2^2 + alphas(2)*trans^2;
M = diag([sigma1_temp, sigma2_temp, sigma3_temp]);
theta = muIn(3);
V = [-trans*sin(theta+rot1), cos(theta+rot1), 0;
      trans*cos(theta+rot1), sin(theta+rot1), 0;
       1, 0, 1];
Q = V*M*V';
% --------------------------------------------
% Prediction step
% --------------------------------------------
% EKF prediction of mean and covariance
predMu = prediction(muIn, motionCommand);
predSigma = G*SigmaIn*G' + Q;

%--------------------------------------------------------------
% Correction step
%--------------------------------------------------------------
% Compute expected observation and Jacobian
zhat = wrapToPi(atan2(landmark_y-predMu(2), landmark_x-predMu(1)) - predMu(3));
q = (landmark_x - predMu(1))^2 + (landmark_y - predMu(2))^2;
H = [(landmark_y - predMu(2))/q, -(landmark_x - predMu(1))/q, -1];
% Innovation / residual covariance
S = H*predSigma*H'+R;
% Kalman gain
K = predSigma*H'/S;

% Correction
v = wrapToPi(z - zhat);
mu = predMu + K * v;
I = eye(length(mu));
Sigma = (I - K * H) * predSigma * (I - K * H)' + K * R * K';
%Sigma = (I - K * H) * predSigma;

%{
sys = [];
sys.A = eye(3);
sys.B = V;
sys.f = @(x) [x(1); x(2)];
sys.H = @(x) [x(1)/(x(1)^2 + x(2)^2)^(1/2), x(2)/(x(1)^2 + x(2)^2)^(1/2);
              x(2)/(x(1)^2 + x(2)^2), -x(1)/(x(1)^2 + x(2)^2)];
sys.h = @(x)  [sqrt(x(1)^2 + x(2)^2); atan2(x(1), x(2))];
sys.Q = 1e-3 * eye(2);
sys.R = diag([0.05^2; 0.01^2]);
%}













