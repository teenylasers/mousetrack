%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Top-level script that invokes the tracker simulator.
% All units are SI.

% Terminology
%  - Track = ground truth of the target's state over time
%  - Detection = raw data from the sensor (radar), ie {r, theta, rdot}
%  - Measurement = a vector that is converted from detection, {x, y, r*rdot}

% Structs
%  - x_axis = {min, max, extent}, in units of m
%  - y_axis = {min, max, extent}, in units of m
%  - radar_coords = {p, R, bearing}, bearing is the angle that generates the rotation
%    matrix R
%  - track = {x, y, vx, vy, wpts}, wpts = waypoints that cubic spline uses to define the
%    track
%  - tracks = a cell array of track structs
%  - detection = {r, theta, rdot}
%  - belief_function = {mu, sig, innov, innov_cov, w}, innov = innovation = pre-filter
%    residual

% Global variables
%  - x_axis, y_axis - set up the extent of the space, in cartesian global frame
%  - radar_coords - relates the polar radar frame to the cartesian global frame
%  - dt - track and measurement time step, in unit of s
%  - FLAGS -

clear all;
global x_axis
global y_axis
global radar_coords
global dt
init_setup();

% Flags
global FLAGS
% tracker
FLAGS.run_kf = 0;         % run tracker using linear kalman filter
FLAGS.debug_kf = 0;       % debug tracker with linear kalman filter
FLAGS.run_ekf = 1;        % run tracker using extended kalman filter
FLAGS.debug_ekf = 0;      % debug tracker with extended kalman filter
FLAGS.run_phd_gmm = 0;    % run tracker using PHD filter with gaussian mixture model
% model
FLAGS.model_accel = 0;    % run EKF with acceleration in the motion model
FLAGS.inject_clutter = 0; % inject clutter into the measurements
% checks
assert(...
  sum([FLAGS.run_kf FLAGS.run_ekf FLAGS.debug_kf FLAGS.debug_ekf FLAGS.run_phd_gmm])==1, ...
  FLAGS);
if FLAGS.model_accel
  assert(~FLAGS.run_kf && ~FLAGS.debug_kf, ...
         'Acceleration model for linear KF is not implemented.');
  assert(~FLAGS.run_phd_gmm, 'Acceleration model for PHD-GMM is not implemented.');
end

% Generate a single ground-truth track
N = 600/15/dt + round(randn(1) * 22);
tracks{1} = generate_track(N);

% Generate radar detections from the ground-truth track at each time step index ti,
% and run filter on it to estimate a track from the detections.
dets = {};
meas = {};
% Represent beliefs using a cell array, where it can be indexed by time, and the
% belief at each time step can consists of a different number of elements (gaussians)
beliefs = {};

for ti = 1:N % ti = time index

  fprintf('\nTime step %d\n', ti);

  % Get the next measurement
  [new_dets, new_meas] = get_next_measurement(tracks, ti);
  if FLAGS.run_kf || FLAGS.run_ekf || FLAGS.run_phd_gmm
    dets{end+1} = new_dets;
  end
  meas{end+1} = new_meas;

  % TODO (jq): inject clutter and noise here

  % Tracking
  if length(beliefs)==0 %|| sum(sum(beliefs{end}.sig))==0
    fprintf('No track initiated yet at time step %d\n', ti);
    if FLAGS.run_kf || FLAGS.run_ekf
      b = initiate_track(new_dets{end}, new_meas{end});
      new_belief = {b};
    elseif FLAGS.debug_kf || FLAGS.debug_ekf
      b = initiate_track(new_dets{end}, new_meas{end});
      new_belief = {b}
    else
      fprintf('PHD filter does not need explicit track initiation.\n');
      [new_belief, predictions] = run_filter(new_meas, new_dets, dt, {}, {});
    end
  else
    [new_belief, predictions] = run_filter(new_meas, new_dets, dt, beliefs{end}, {});
  end
  beliefs{end+1} = new_belief;

  % Step-by-step visualization
  if mod(ti,10)==0
    visualize_tracker_results(radar_coords, tracks, dets, meas, beliefs);
    %xlim([x_axis.min-x_axis.extent*0.1 x_axis.max+x_axis.extent*0.1]);
    %ylim([y_axis.min-y_axis.extent*0.1 y_axis.max+y_axis.extent*0.1]);
    disp('Press any key to continue.\n'); pause;
  end
end

% Evaluate and visualize how well the filter does
if FLAGS.run_kf || FLAGS.debug_kf || FLAGS.run_ekf || FLAGS.debug_ekf
  evaluate_kalman_filter(tracks(1), meas, beliefs, 1);
else
  assert(false, 'PHD filter eval is not yet implemented.');
  % evaluate_phd_filter();
end

% Visualize the track(s) and measurements
visualize_tracker_results(radar_coords, tracks, dets, meas, beliefs);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper functions

%%%  Helper function generate_track
function track = generate_track(N)
global x_axis
global y_axis
global dt
global FLAGS
if FLAGS.debug_kf || FLAGS.debug_ekf
  track = track_generator_debug(x_axis, y_axis, N, dt);
else
  track = track_generator(x_axis, y_axis, N, dt);
end
end % function generate_track

%%%  Helper function get_next_measurement
function [new_dets, new_meas] = get_next_measurement(tracks, ti)
global radar_coords
global FLAGS

if length(tracks)>1
  assert(FLAGS.run_phd_gmm, 'Multi-target tracking not yet implemented for (E)KF.');
end

if FLAGS.debug_kf
  d.r = nan;
  d.theta = nan;
  d.rdot = nan;
  m = measurement_generator_kf_debug(tracks{1}, ti);
  new_dets = {d};
  new_meas = {m};
elseif FLAGS.debug_ekf
  d.r = nan;
  d.theta = nan;
  d.rdot = nan;
  m = measurement_generator_ekf_debug(tracks{1}, ti);
  new_dets = {d};
  new_meas = {m};
else
  new_dets = detections_generator(tracks, ti, radar_coords);
  if FLAGS.run_kf
    new_meas = kf_convert_detection_to_measurement(new_dets);
  elseif FLAGS.run_ekf || FLAGS.run_phd_gmm
    new_meas = ekf_convert_detection_to_measurement(new_dets);
  end
end

end % function get_next_measurement

%%%  Helper function run_filter
function [new_belief, predictions] = run_filter(new_meas, new_dets, dt, prev_belief, ...
                                                future_times)
global FLAGS

% (E)KF only runs on a single target and single track. PHD runs on multiple targets plus
% clutter at the same time. Until multiple target (E)KF is implement, we will turn the
% cell array of multiple beliefs, detections, and measurements into a single struct for
% FLAGS.run/debug_(e)kf
if FLAGS.debug_kf || FLAGS.run_kf
  [nb, predictions] = kf(new_meas{end}, new_dets{end}, dt, prev_belief{end}, future_times);
  new_belief = {nb};
elseif FLAGS.debug_ekf || FLAGS.run_ekf
  [nb, predictions] = ekf(new_meas{end}, new_dets{end}, dt, prev_belief{end}, future_times);
  new_belief = {nb};
elseif FLAGS.run_phd_gmm
  [new_belief, predictions] = phd_gmm(new_dets, dt, prev_belief, future_times);
end
end % function run_filter
