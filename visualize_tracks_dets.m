%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization: overlay detections in the radar frame on track in the global frame
function visualize_tracks_det(tracks, det, radar_coords)

figure; hold on;

% Plot the tracks in the global frame
for i = 1:length(tracks)
  tr = tracks(i);
  plot(tr.wpts.x, tr.wpts.y, '.', tr.x, tr.y, '.');
  quiver(tr.x, tr.y, tr.vx, tr.vy);
end

% Transform detections for plot visualization
all_det = transform_detections(det, radar_coords);

% Plot the noisy and noise-free detections
h1 = polar(all_det.theta + radar_coords.bearing, all_det.r, 'o');
h0 = polar(all_det.theta0 + radar_coords.bearing, all_det.r0, 'x');
plot_rdot = 1;
if plot_rdot
  det_colour = get(h1, 'Color');
  quiver(all_det.xy(1,:), all_det.xy(2,:), all_det.vxy(1,:), all_det.vxy(2,:), ...
      'Color', det_colour, 'AutoScaleFactor', 5);
  det0_color = get(h0, 'Color');
  quiver(all_det.xy0(1,:), all_det.xy0(2,:), all_det.vxy0(1,:), all_det.vxy0(2,:), ...
      'Color', det0_color, 'AutoScaleFactor', 10);
end
% TODO: plot error-free detections with covariances.

legend('track waypoints', 'track true path', 'track true velocity', ...
    'noisy detections', 'noise-free detections', 'Location', 'northwest');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tests:
%
% Change the 2D space in global coords, i.e. change x_range.min and y_range.min
% Change radar location and bearing.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function accumulateDet4Plot
%
% Transform time sequenced detections for plotting and visualization
%

function all_det = transform_detections(det, radar_coords)

% all_det.r = zeros(1, length(det));
% all_det.r_dot = zeros(1, length(det));
% all_det.theta = zeros(1, length(det));
% all_det.xy = zeros(2, length(det));
% all_det.vxy = zeros(2, length(det));
for i = 1:length(det)
  all_det.r(i) = det(i).r;
  all_det.r_dot(i) = det(i).r_dot;
  all_det.theta(i) = det(i).theta;
  all_det.xy(:,i) = radar_coords.p + ...
      det(i).r * radar_coords.R * [cos(det(i).theta); sin(det(i).theta)];
  all_det.vxy(:,i) = radar_coords.p + ...
      det(i).r_dot * radar_coords.R * [cos(det(i).theta); sin(det(i).theta)];

  all_det.r0(i) = det(i).r0;
  all_det.r_dot0(i) = det(i).r_dot0;
  all_det.theta0(i) = det(i).theta0;
  all_det.xy0(:,i) = radar_coords.p + ...
      det(i).r0 * radar_coords.R * [cos(det(i).theta0); sin(det(i).theta0)];
  all_det.vxy0(:,i) = radar_coords.p + ...
      det(i).r_dot0 * radar_coords.R * [cos(det(i).theta0); sin(det(i).theta0)];
end
