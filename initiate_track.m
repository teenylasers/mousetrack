%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function initiate_track
%
% Given a detection and the corresponding measurement vector, return a belief function
% that represents a new track. Decisions on whether this detection *ought* to initiate a
% new track is made elsewhere.
%
% Input:
% d = a detection
% m = the corresponding measurement vector

function belief = initiate_track(d, m)

global FLAGS

if FLAGS.run_kf || FLAGS.run_ekf
  r = d.r;
  theta = d.theta;
  rdot = d.rdot;
  if isnan(r) || isnan(theta) || isnan(rdot)
    fprintf('Erroneous input: r = %f, theta = %f, rdot = %f\n', ...
            r, theta, rdot);
    return;
  end
elseif FLAGS.debug_kf || FLAGS.debug_ekf
  if sum(isnan(m(end)))
    fprintf('Erroneous input: dets(end) = %f', dets(end));
    return
  end
end

if FLAGS.run_kf
  belief.mu = kf_measurement_to_initial_state(d);
elseif FLAGS.debug_kf
  belief.mu = kf_measurement_to_initial_state(m);
elseif FLAGS.run_ekf
  belief.mu = ekf_measurement_to_initial_state(d);
elseif FLAGS.debug_ekf
  belief.mu = ekf_measurement_to_initial_state(m);
end

belief.sig = 1e9 * eye(length(belief.mu));
belief.innov = zeros(length(m), 1);
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
%   curr_meas = convert_detection_to_measurement(det);
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