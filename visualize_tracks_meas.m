%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization: overlay measurements in the radar frame on track in the global frame
function visualize_tracks_meas(tracks, meas, radar_coords)

figure; hold on;

% Plot the tracks in the global frame
for i = 1:length(tracks)
  tr = tracks(i);
  plot(tr.wpts.x, tr.wpts.y, '.', tr.x, tr.y, '.');
  quiver(tr.x, tr.y, tr.vx, tr.vy);
end

% Transform measurements for plot visualization
all_meas = transform_measurements(meas, radar_coords);

% Plot the noisy and noise-free measurements
h1 = polar(all_meas.theta + radar_coords.bearing, all_meas.r, 'o');
h0 = polar(all_meas.theta0 + radar_coords.bearing, all_meas.r0, 'x');
plot_rdot = 1;
if plot_rdot
  meas_colour = get(h1, 'Color');
  quiver(all_meas.xy(1,:), all_meas.xy(2,:), all_meas.vxy(1,:), all_meas.vxy(2,:), ...
      'Color', meas_colour);
  meas0_color = get(h0, 'Color');
  quiver(all_meas.xy0(1,:), all_meas.xy0(2,:), all_meas.vxy0(1,:), all_meas.vxy0(2,:), ...
      'Color', meas0_color);
end
% TODO: plot error-free measurements with covariances.

legend('track waypoints', 'track true path', 'track true velocity', ...
    'noisy measurements', 'noise-free measurements', 'Location', 'northwest');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tests:
%
% Change the 2D space in global coords, i.e. change x_range.min and y_range.min
% Change radar location and bearing.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function accumulateMeas4Plot
%
% Transform time sequenced measurements for plotting and visualization
%

function all_meas = transform_measurements(meas, radar_coords)

% all_meas.r = zeros(1, length(meas));
% all_meas.r_dot = zeros(1, length(meas));
% all_meas.theta = zeros(1, length(meas));
% all_meas.xy = zeros(2, length(meas));
% all_meas.vxy = zeros(2, length(meas));
for i = 1:length(meas)
  all_meas.r(i) = meas(i).r;
  all_meas.r_dot(i) = meas(i).r_dot;
  all_meas.theta(i) = meas(i).theta;
  all_meas.xy(:,i) = radar_coords.p + ...
      meas(i).r * radar_coords.R * [cos(meas(i).theta); sin(meas(i).theta)];
  all_meas.vxy(:,i) = radar_coords.p + ...
      meas(i).r_dot * radar_coords.R * [cos(meas(i).theta); sin(meas(i).theta)];

  all_meas.r0(i) = meas(i).r0;
  all_meas.r_dot0(i) = meas(i).r_dot0;
  all_meas.theta0(i) = meas(i).theta0;
  all_meas.xy0(:,i) = radar_coords.p + ...
      meas(i).r0 * radar_coords.R * [cos(meas(i).theta0); sin(meas(i).theta0)];
  all_meas.vxy0(:,i) = radar_coords.p + ...
      meas(i).r_dot0 * radar_coords.R * [cos(meas(i).theta0); sin(meas(i).theta0)];
end
