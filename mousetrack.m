% Top-level script that invokes the tracker simulator.
% All units are SI.

% Terminology
% Track = ground truth of the target's state over time
% Detection = raw data from the sensor (radar), ie {r, theta, r_dot}
% Measurement = a vector that is converted from detection, {x, y, r*r_dot}

% Structs
% x_axis = {min, max, extent}, in units of m
% y_axis = {min, max, extent}, in units of m
% radar_coords = {p, R, bearing}, bearing is the angle that generates the rotation matrix R
% track = {x, y, vx, vy, wpts}, wpts = waypoints that cubic spline uses to define the track
% detection = {r, theta, r_dot}
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

% Generate a single ground-truth track
N = 600/15/dt + round(randn(1) * 22);
tracks(1) = track_generator(x_axis, y_axis, N, dt);

% Generate radar detections from a track for time indices ti
dets = [];
beliefs = [];
for ti = 1:N
  new_dets = detections_generator(tracks, ti, radar_coords);
  dets = [dets new_dets];
  new_meas = convert_detection_to_measurement(new_dets);
  if length(beliefs)==0 || sum(sum(beliefs(end).sig))==0
    fprintf('No track initiated yet at time step %d\n', ti);
    new_belief = initiate_track(dets);
  else
    [new_belief, predictions] = kalman_filter(new_meas, new_dets, dt, beliefs(end), []);
  end
  beliefs = [beliefs new_belief];
  % visualize_tracker_results(radar_coords, tracks, dets, beliefs);
  % xlim([x_axis.min-x_axis.extent*0.1 x_axis.max+x_axis.extent*0.1]);
  % ylim([y_axis.min-y_axis.extent*0.1 y_axis.max+y_axis.extent*0.1]);
  % disp('Press any key to continue.\n'); pause;
end
evaluate_kalman_filter(tracks(1), dets, beliefs, 1);

% Visualize the track(s) and measurements
visualize_tracker_results(radar_coords, tracks, dets, beliefs);
xlim([x_axis.min-x_axis.extent*0.1 x_axis.max+x_axis.extent*0.1]);
ylim([y_axis.min-y_axis.extent*0.1 y_axis.max+y_axis.extent*0.1]);