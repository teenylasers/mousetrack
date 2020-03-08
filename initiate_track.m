%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function initiate_track
%
% Returns a belief function belief that represents a new track.
%
% Input:
% dets = an array of detections {r, theta, r_dot}, the last element is the latest
% detection. If FLAGS.debug_*, then dets is actually an array of measurements, detection
% to measurement step is bypassed in debug mode.

function belief = initiate_track(FLAGS, dets)

if FLAGS.run_kf || FLAGS.run_ekf
  r = dets(end).r;
  theta = dets(end).theta;
  r_dot = dets(end).r_dot;
  if isnan(r) || isnan(theta) || isnan(r_dot)
    fprintf('Erroneous input: r = %f, theta = %f, r_dot = %f\n', ...
            r, theta, r_dot);
    return;
  end
elseif FLAGS.debug_kf || FLAGS.debug_ekf
  if sum(isnan(dets(end)))
    fprintf('Erroneous input: dets(end) = %f', dets(end));
    return
  end
end

if FLAGS.debug_ekf
  r = dets(1, end)
  theta = dets(2, end)
  r_dot = dets(3, end)
end

if FLAGS.run_kf
  m = kf_convert_detection_to_measurement(dets(end));
  xdot = -r_dot * sin(theta);
  ydot = r_dot * cos(theta);
  belief.mu = [m(1); xdot; m(2); ydot];
  belief.sig = 1e9 * eye(length(belief.mu));
  belief.innov = zeros(length(m), 1);
  belief.innov_cov = eye(length(belief.innov));
elseif FLAGS.run_ekf || FLAGS.debug_ekf
  x = -r * sin(theta);
  y = r * cos(theta);
  xdot = -r_dot * sin(theta);
  ydot = r_dot * cos(theta);
  belief.mu = [x; xdot; y; ydot];
  belief.sig = 1e9 * eye(length(belief.mu));
  belief.innov = zeros(length(dets(end)), 1);
  belief.innov_cov = eye(length(belief.innov));
elseif FLAGS.debug_kf
  x = dets(1, end);
  xdot = 0;
  y = dets(2, end);
  ydot = 0;
  belief.mu = [x; xdot; y; ydot];
  belief.sig = 1e9 * eye(length(belief.mu));
  belief.innov = zeros(length(dets(end)),1);
  belief.innov_cov = eye(length(belief.innov));
end

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

end
