function varargout = run(stepsOrData, pauseLen, makeVideo)
% RUN  PS2 Feature-Based Localization Simulator
%   RUN(ARG)
%   RUN(ARG, PAUSLEN, MAKEVIDEO)
%      ARG - is either the number of time steps, (e.g. 100 is a complete
%            circuit) or a data array from a previous run.
%      PAUSELEN - set to `inf`, to manually pause, o/w # of seconds to wait
%                 (e.g., 0.3 is the default)
%      MAKEVIDEO - boolean specifying whether to record a video or not
%
%   DATA = RUN(ARG,PAUSELEN)
%      DATA - is an optional output and contains the data array generated
%             and/or used during the simulation.

%   (c) 2009-2015
%   Ryan M. Eustice
%   University of Michigan
%   eustice@umich.edu

if ~exist('pauseLen','var') || isempty(pauseLen)
    pauseLen = 0.3; % seconds
end
if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end

%--------------------------------------------------------------
% Graphics
%--------------------------------------------------------------

NOISEFREE_PATH_COL = 'green';
ACTUAL_PATH_COL = 'blue';

NOISEFREE_BEARING_COLOR = 'cyan';
OBSERVED_BEARING_COLOR = 'red';

GLOBAL_FIGURE = 1;

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(5, 1/pauseLen));
        open(vo);
    end
end

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------

initialStateMean = [180 50 0]';

% Motion noise (in odometry space, see Table 5.5, p.134 in book).
alphas = [0.05 0.001 0.05 0.01].^2; % variance of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
beta = deg2rad(20);

% Step size between filter updates, can be less than 1.
deltaT=0.1;

persistent data numSteps;
if isempty(stepsOrData) % use dataset from last time
    if isempty(data)
        numSteps = 100;
        data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
    end
elseif isscalar(stepsOrData)
    % Generate a dataset of motion and sensor info consistent with
    % noise models.
    numSteps = stepsOrData;
    data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
else
    % use a user supplied dataset from a previous run
    data = stepsOrData;
    numSteps = size(data, 1);
    global FIELDINFO;
    FIELDINFO = getfieldinfo;
end


% TODO: provide proper initialization for your filters here
% You can set the initial mean and variance of the EKF to the true mean and
% some uncertainty.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%My Code Here%%%%%%%%%%%%%%%%%%%%%%
%%%%% Initialize all the variable 
G = eye(3);
mu = initialStateMean;
%mu_x = rand*700 -100;
%mu_y = rand*400 -100;
%mu = [mu_x, mu_y, 0]';
sigma = 200*eye(3);
Q = beta^2;
% Call ekfUpdate, ukfUpdate and pfUpdate in every iteration of this loop.
% You might consider putting in a switch yard so you can select which
% algorithm does the update
results = [];
for t = 1:numSteps

    %=================================================
    % data available to your filter at this time step
    %=================================================
    % the input is angle and distance, not velocity
    motionCommand = data(t,3:5)'; % [drot1, dtrans, drot2]' noisefree control command
    observation = data(t,1:2)';   % [bearing, landmark_id]' noisy observation
    
    %=================================================
    % data *not* available to your filter, i.e., known
    % only by the simulator, useful for making error plots
    %=================================================
    % actual position (i.e., ground truth)
    x = data(t,8);
    y = data(t,9);
    theta = data(t,10);

    % noisefree observation
    noisefreeBearing = data(t, 6);

    %=================================================
    % graphics
    %=================================================
    figure(GLOBAL_FIGURE); clf; hold on; plotfield(observation(2));

    % draw actual path and path that would result if there was no noise in
    % executin the motion command
    plot([initialStateMean(1) data(1,8)], [initialStateMean(2) data(1,9)], 'Color', ACTUAL_PATH_COL);
    plot([initialStateMean(1) data(1,11)], [initialStateMean(2) data(1,12)], 'Color', NOISEFREE_PATH_COL);

    % draw actual path (i.e., ground truth)
    plot(data(1:t,8), data(1:t,9), 'Color', ACTUAL_PATH_COL);
    plotrobot( x, y, theta, 'black', 1, 'cyan');

    % draw noise free motion command path
    plot(data(1:t,11), data(1:t,12), 'Color', NOISEFREE_PATH_COL);
    plot(data(t,11), data(t,12), '*', 'Color', NOISEFREE_PATH_COL);

    % indicate observed angle relative to actual position
    plot([x x+cos(theta+observation(1))*100], [y y+sin(theta+observation(1))*100], 'Color', OBSERVED_BEARING_COLOR);

    % indicate ideal noise-free angle relative to actual position
    plot([x x+cos(theta+noisefreeBearing)*100], [y y+sin(theta+noisefreeBearing)*100], 'Color', NOISEFREE_BEARING_COLOR);

    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
    %mu_draw = mu(1:2);
    %sigma_draw = sigma(1:2,1:2)*3;
    %draw_ellipse(mu_draw, sigma_draw, 9, 'r')
    
    
    z = observation(1);
    markedId = observation(2);
    
    [mu, sigma, preMu, preSigma] = ekfUpdate(mu, sigma, motionCommand, alphas, G, z, Q, markedId);
    mu_draw = mu(1:2);
    sigma_draw = sigma(1:2,1:2)*3;
    draw_ellipse(mu_draw, sigma_draw, 9, 'b')
    %=================================================
    %TODO: plot and evaluate filter results here
    %=================================================
    



    drawnow;
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end

    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
end

if nargout >= 1
    varargout{1} = data;
end
if nargout >= 2
    varargout{2} = results;
end

if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end
