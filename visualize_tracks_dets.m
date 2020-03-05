%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization: overlay detections in the radar frame on track in the global frame
function visualize_tracks_dets(tracks, dets, radar_coords, new_fig, hold_on)

if new_fig
  figure;
end

% Plot the tracks in the global frame
for i = 1:length(tracks)
  tr = tracks(i);
  plot(tr.wpts.x, tr.wpts.y, '.', tr.x, tr.y, '.');
  hold on;
  quiver(tr.x, tr.y, tr.vx, tr.vy);
end

% Transform detections for plot visualization
all_dets = transform_detections(dets, radar_coords);

% Plot the noisy and noise-free detections
h1 = polar(all_dets.theta + radar_coords.bearing, all_dets.r, 'o');
h0 = polar(all_dets.theta0 + radar_coords.bearing, all_dets.r0, 'x');
plot_rdot = 1;
if plot_rdot
  det_colour = get(h1, 'Color');
  quiver(all_dets.xy(1,:), all_dets.xy(2,:), all_dets.vxy(1,:), all_dets.vxy(2,:), ...
      'Color', det_colour);
  det0_color = get(h0, 'Color');
  quiver(all_dets.xy0(1,:), all_dets.xy0(2,:), all_dets.vxy0(1,:), all_dets.vxy0(2,:), ...
      'Color', det0_color);
end
% TODO: plot error-free detections with covariances.

legend('track waypoints', 'track true path', 'track true velocity', ...
    'noisy detections', 'noise-free detections', 'Location', 'northwest');
if ~hold_on
  hold off;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tests:
%
% Change the 2D space in global coords, i.e. change x_axis.min and y_axis.min
% Change radar location and bearing.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function transform_detections
%
% Transform time sequenced detections for plotting and visualization
%

function all_dets = transform_detections(dets, radar_coords)

% all_dets.r = zeros(1, length(dets));
% all_dets.r_dot = zeros(1, length(dets));
% all_dets.theta = zeros(1, length(dets));
% all_dets.xy = zeros(2, length(dets));
% all_dets.vxy = zeros(2, length(dets));
for i = 1:length(dets)
  all_dets.r(i) = dets(i).r;
  all_dets.r_dot(i) = dets(i).r_dot;
  all_dets.theta(i) = dets(i).theta;
  all_dets.xy(:,i) = radar_coords.p + ...
      dets(i).r * radar_coords.R * [cos(dets(i).theta); sin(dets(i).theta)];
  all_dets.vxy(:,i) = radar_coords.p + ...
      dets(i).r_dot * radar_coords.R * [cos(dets(i).theta); sin(dets(i).theta)];

  all_dets.r0(i) = dets(i).r0;
  all_dets.r_dot0(i) = dets(i).r_dot0;
  all_dets.theta0(i) = dets(i).theta0;
  all_dets.xy0(:,i) = radar_coords.p + ...
      dets(i).r0 * radar_coords.R * [cos(dets(i).theta0); sin(dets(i).theta0)];
  all_dets.vxy0(:,i) = radar_coords.p + ...
      dets(i).r_dot0 * radar_coords.R * [cos(dets(i).theta0); sin(dets(i).theta0)];
end
