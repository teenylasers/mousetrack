% Top-level script that invokes the tracker simulator.
% All units are SI.

% Terminology
% Track = ground truth of the target's state over time
% Detection = raw data from the sensor (radar), ie {r, theta, rdot}
% Measurement = a vector that is converted from detection, {x, y, r*rdot}

% Structs
% x_axis = {min, max, extent}, in units of m
% y_axis = {min, max, extent}, in units of m
% radar_coords = {p, R, bearing}, bearing is the angle that generates the rotation matrix R
% track = {x, y, vx, vy, wpts}, wpts = waypoints that cubic spline uses to define the track
% detection = {r, theta, rdot}
% belief_function = {mu, sig, innov, innov_cov}, innov = innovation = pre-filter residual

% Global variables
% x_axis, y_axis - set up the extent of the space, in cartesian global frame
% radar_coords - relates the polar radar frame to the cartesian global frame
% dt - track and measurement time step, in unit of s

clear all;
global x_axis
global y_axis
global radar_coords
global dt
init_setup();

% Flags
% Only one of the following should be >0.
global FLAGS
FLAGS.run_kf = 0;         % run tracker using linear kalman filter
FLAGS.debug_kf = 0;       % debug tracker with linear kalman filter
FLAGS.run_ekf = 1;        % run tracker using extended kalman filter
FLAGS.debug_ekf = 0;      % debug tracker with extended kalman filter
FLAGS.model_accel = 0;      % run EKF with acceleration in the model
assert(sum([FLAGS.run_kf FLAGS.run_ekf FLAGS.debug_kf FLAGS.debug_ekf])==1, FLAGS);
if FLAGS.model_accel
  assert(~FLAGS.run_kf && ~FLAGS.debug_kf, ...
         'Constant acceleration model for linear KF is not implemented.');
end

% Generate a single ground-truth track
N = 600/15/dt + round(randn(1) * 22);
tracks(1) = generate_track(N);

% Generate radar detections from a track for time indices ti
dets = [];
meas = [];
beliefs = [];
for ti = 1:N % ti = time index
  % Get the next measurement
  [new_dets, new_meas] = get_next_measurement(tracks(1), ti);
  if FLAGS.run_kf || FLAGS.run_ekf
    dets = [dets new_dets];
  end
  meas = [meas new_meas];

  if length(beliefs)==0 || sum(sum(beliefs(end).sig))==0
    %fprintf('No track initiated yet at time step %d\n', ti);
    if FLAGS.run_kf || FLAGS.run_ekf
      new_belief = initiate_track(dets);
    else
      new_belief = initiate_track(meas);
    end
  else
    [new_belief, predictions] = run_filter(new_meas, new_dets, dt, beliefs(end), []);
  end
  beliefs = [beliefs new_belief];

  % Step-by-step visualization
  % visualize_tracker_results(radar_coords, tracks, dets, beliefs);
  % xlim([x_axis.min-x_axis.extent*0.1 x_axis.max+x_axis.extent*0.1]);
  % ylim([y_axis.min-y_axis.extent*0.1 y_axis.max+y_axis.extent*0.1]);
  % disp('Press any key to continue.\n'); pause;
end
evaluate_kalman_filter(tracks(1), meas, beliefs, 1);

% Visualize the track(s) and measurements
visualize_tracker_results(radar_coords, tracks, dets, meas, beliefs);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper functions

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

function [new_dets, new_meas] = get_next_measurement(track, ti)
global radar_coords
global FLAGS
if FLAGS.debug_kf
  new_dets.r = nan;
  new_dets.theta = nan;
  new_dets.rdot = nan;
  new_meas = measurement_generator_kf_debug(track, ti);
elseif FLAGS.debug_ekf
  new_dets.r = nan;
  new_dets.theta = nan;
  new_dets.rdot = nan;
  new_meas = measurement_generator_ekf_debug(track, ti);
else
  new_dets = detections_generator(track, ti, radar_coords);
  if FLAGS.run_kf
    new_meas = kf_convert_detection_to_measurement(new_dets);
  elseif FLAGS.run_ekf
    new_meas = ekf_convert_detection_to_measurement(new_dets);
  end
end
end % function get_next_measurement

function [new_belief, predictions] = run_filter(new_meas, new_dets, dt, prev_belief, ...
                                                future_times)
global FLAGS
if FLAGS.debug_kf || FLAGS.run_kf
  [new_belief, predictions] = kf(new_meas, new_dets, dt, prev_belief, future_times);
elseif FLAGS.debug_ekf || FLAGS.run_ekf
  [new_belief, predictions] = ekf(new_meas, new_dets, dt, prev_belief, future_times);
end
end % function run_filter
