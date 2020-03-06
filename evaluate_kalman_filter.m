%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function evaluate_kalman_filter
%
% For an array of belief functions that represent a time sequence of state estimation,
% evaluate and visualize the goodness of the Kalman filter.
%
% Input:
% true_track = ground-truth track
% detections = time sequence of detections
% beliefs = time sequence of belief functions that estimate the tracks
%

function [position_err_sq, velocity_err_sq] = evaluate_kalman_filter(...
    true_track, detections, beliefs, plot_res)

time_length = length(true_track.x);
assert(length(beliefs) == time_length);

% 1. position and velocity errors squared
naive_p_err = zeros(1, time_length);
p_err = zeros(1, time_length);
v_err = zeros(1, time_length);
for i = 1:time_length
  m = convert_detection_to_measurement(detections(i));
  naive_dx = true_track.x(i) - m(1);
  naive_dy = true_track.y(i) - m(2);
  dx = true_track.x(i) - beliefs(i).mu(1);
  dy = true_track.y(i) - beliefs(i).mu(3);
  dvx = true_track.vx(i) - beliefs(i).mu(2);
  dvy = true_track.vy(i) - beliefs(i).mu(4);
  p_err(i) = sqrt(dx^2 + dy^2);
  v_err(i) = sqrt(dvx^2 + dvy^2);
  naive_p_err(i) = sqrt(naive_dx^2 + naive_dy^2);
end
if plot_res
  figure;
  subplot(2,1,1);
  plot(1:time_length, p_err, 1:time_length, naive_p_err);
  legend('filtered', 'unfiltered');
  xlabel('Time step'); ylabel('Position error abs val (m)');
  subplot(2,1,2);
  plot(v_err);	
  xlabel('Time step'); ylabel('Velocity error abs val (m/s)');
end

% 2. plot innovation
x_innov = zeros(1, time_length);
x_innov_bound = zeros(1, time_length);
y_innov = zeros(1, time_length);
y_innov_bound = zeros(1, time_length);
rrdot_innov = zeros(1, time_length);
rrdot_innov_bound = zeros(1, time_length);
for i = 1:time_length
  % TODO: only plotting covariance diagonal terms, how to use off-diag terms.
  x_innov(i) = beliefs(i).innov(1);
  x_innov_bound(i) = sqrt(beliefs(i).innov_cov(1,1));
  y_innov(i) = beliefs(i).innov(2);
  y_innov_bound(i) = sqrt(beliefs(i).innov_cov(2,2));
  rrdot_innov(i) = beliefs(i).innov(3);
  rrdot_innov_bound(i) = sqrt(beliefs(i).innov_cov(3,3));
end
if plot_res
  figure;
  title('Measurement residual');
  subplot(3,1,1);
  plot(1:time_length, x_innov, ...
      1:time_length, x_innov_bound, 1:time_length, -1 * x_innov_bound);
  xlabel('Time step'); ylabel('x (m)');
  subplot(3,1,2);
  plot(1:time_length, y_innov, ...
      1:time_length, y_innov_bound, 1:time_length, -1 * y_innov_bound);
  xlabel('Time step'); ylabel('y (m)');
  subplot(3,1,3);
  plot(1:time_length, rrdot_innov, ...
      1:time_length, rrdot_innov_bound, 1:time_length, -1 * rrdot_innov_bound);
  ylim([-500 500]);
  xlabel('Time step'); ylabel('r*rdot (m)');
end

% 3. normalized innovations squared chi2

% 4. auto-correlation to check residual is white