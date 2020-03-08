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
    true_track, meas, beliefs, plot_res)

time_length = length(true_track.x);
assert(length(beliefs) == time_length);

% 1. position and velocity errors squared
naive_p_err = zeros(1, time_length);
p_err = zeros(1, time_length);
v_err = zeros(1, time_length);
for i = 1:time_length
  naive_dx = true_track.x(i) - meas(1,i);
  naive_dy = true_track.y(i) - meas(2,i);
  dx = true_track.x(i) - beliefs(i).mu(1);
  dy = true_track.y(i) - beliefs(i).mu(3);
  dvx = true_track.vx(i) - beliefs(i).mu(2);
  dvy = true_track.vy(i) - beliefs(i).mu(4);
  p_err(i) = sqrt(dx^2 + dy^2);
  v_err(i) = sqrt(dvx^2 + dvy^2);
  naive_p_err(i) = sqrt(naive_dx^2 + naive_dy^2);
end

belief_x = zeros(1, time_length);
belief_y = zeros(1, time_length);
belief_vx = zeros(1, time_length);
belief_vy = zeros(1, time_length);
for i = 1:time_length
  belief_x(i) = beliefs(i).mu(1);
  belief_y(i) = beliefs(i).mu(3);
  belief_vx(i) = beliefs(i).mu(2);
  belief_vy(i) = beliefs(i).mu(4);
end

if plot_res
  x_colour = [0 0.4470 0.7410];
  y_colour = [0.8500 0.3250 0.0980];

  figure;
  title('Track estimation error');
  subplot(2,2,1);
  plot(1:time_length, p_err, 1:time_length, naive_p_err);
  legend('filtered', 'unfiltered');
  xlabel('Time step'); ylabel('Position error abs val (m)');
  subplot(2,2,2); hold on;
  plot(1:time_length, belief_x, '-', ...
      1:time_length, true_track.x, '--', 'Color', x_colour);
  plot(1:time_length, belief_y, '-', ...
      1:time_length, true_track.y, '--', 'Color', y_colour);
  hold off;
  legend('Estimated x', 'Ground truth x', 'Estimated y', 'Ground truth y', ...
      'Location', 'SouthEast');
  xlabel('Time step'); ylabel('{x, y} estimate error (m)');
  subplot(2,2,3);
  plot(v_err);
  ylim([0 20]);
  xlabel('Time step'); ylabel('Velocity error abs val (m/s)');
  subplot(2,2,4); hold on;
  plot(1:time_length, belief_vx, '-', ...
      1:time_length, true_track.vx, '--', 'Color', x_colour);
  plot(1:time_length, belief_vy, '-', ...
      1:time_length, true_track.vy, '--', 'Color', y_colour);
  hold off; ylim([-30 30]);
  legend('Estimated vx', 'Ground truth vx', 'Estimated vy', 'Ground truth vy', ...
      'Location', 'SouthEast');
  xlabel('Time step'); ylabel('{x dot, y dot} estimate error (m/s)');
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
  ylim([-50 50]);
  xlabel('Time step'); ylabel('x (m)');
  subplot(3,1,2);
  plot(1:time_length, y_innov, ...
      1:time_length, y_innov_bound, 1:time_length, -1 * y_innov_bound);
  ylim([-50 50]);
  xlabel('Time step'); ylabel('y (m)');
  subplot(3,1,3);
  plot(1:time_length, rrdot_innov, ...
      1:time_length, rrdot_innov_bound, 1:time_length, -1 * rrdot_innov_bound);
  ylim([-500 500]);
  xlabel('Time step'); ylabel('r*rdot (m)');
end

% 3. normalized innovations squared chi2

% 4. auto-correlation to check residual is white
