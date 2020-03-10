%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function initiate_track
%
% Returns a belief function belief that represents a new track.
%
% Input:
% dets = an array of detections {r, theta, rdot}, the last element is the latest
% detection. If FLAGS.debug_*, then dets is actually an array of measurements, detection
% to measurement step is bypassed in debug mode.

function belief = initiate_track(dets)

global FLAGS

if FLAGS.run_kf || FLAGS.run_ekf
  r = dets(end).r;
  theta = dets(end).theta;
  rdot = dets(end).rdot;
  if isnan(r) || isnan(theta) || isnan(rdot)
    fprintf('Erroneous input: r = %f, theta = %f, rdot = %f\n', ...
            r, theta, rdot);
    return;
  end
elseif FLAGS.debug_kf || FLAGS.debug_ekf
  if sum(isnan(dets(end)))
    fprintf('Erroneous input: dets(end) = %f', dets(end));
    return
  end
end

if FLAGS.run_kf
  m = kf_convert_detection_to_measurement(dets(end));
  x = m(1);
  y = m(2);
  xdot = -rdot * sin(theta);
  ydot = rdot * cos(theta);
  belief.innov = zeros(length(m), 1);
elseif FLAGS.debug_kf
  x = dets(1, end);
  xdot = 0;
  y = dets(2, end);
  ydot = 0;
  belief.innov = zeros(size(dets,1),1);
elseif FLAGS.run_ekf
  m = ekf_convert_detection_to_measurement(dets(end));
  x = -r * sin(theta);
  y = r * cos(theta);
  xdot = -rdot * sin(theta);
  ydot = rdot * cos(theta);
  belief.innov = zeros(length(m), 1);
elseif FLAGS.debug_ekf
  r = dets(1, end);
  theta = dets(2, end);
  rdot = dets(3, end);
  x = -r * sin(theta);
  y = r * cos(theta);
  xdot = 0;
  ydot = 0;
  belief.innov = zeros(size(dets, 1), 1);
end

if FLAGS.model_accel
  xdotdot = 0;
  ydotdot = 0;
  belief.mu = [x; xdot; xdotdot; y; ydot; ydotdot];
else
  belief.mu = [x; xdot; y; ydot];
end
belief.sig = 1e9 * eye(length(belief.mu));
belief.innov_cov = eye(length(belief.innov));

% Decide whether there has been enough detections to initiate a track.
% if length(dets) < 2
%   can_init_track = 0;
% else
%   can_init_track = 1;
% end
%
% % If cannot initiate a track, then return an all-zero belief
% if ~can_init_track
%   belief.mu = zeros(4,1);
%   belief.sig = zeros(4);
%   belief.innov = zeros(3,1);
%   belief.innov_cov = zeros(3);
% else
%   prev_meas = convert_detection_to_measurement(dets(length(dets)-1));
%   curr_meas = convert_detection_to_measurement(dets(end));
%   global dt
%   xdot = (curr_meas(1)-prev_meas(1))/dt;
%   ydot = (curr_meas(2)-prev_meas(2))/dt;
%   belief.mu = [curr_meas(1); xdot; curr_meas(2); ydot];
%   belief.sig = [10  0  0  0;
%                  0  0.1  0  0;
% 		 0  0  10  0;
% 		 0  0  0  0.1]; % TODO: not correct
%   belief.innov = [0; 0; 0];
%   belief.innov_cov = zeros(3);
% end

end % function initiate_track