%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization: overlay detections in the radar frame on track in the global frame
function visualize_tracker_results(radar_coords, tracks, dets, beliefs)

figure; hold on;

% Plot the tracks in the global frame
for i = 1:length(tracks)
  tr = tracks(i);
  plot(tr.x, tr.y, '.');
  % quiver(tr.x, tr.y, tr.vx, tr.vy);
end

% Plot detections over time in the global frame
all_dets = transform_detections(dets, radar_coords);
h1 = polar(all_dets.theta + radar_coords.bearing, all_dets.r, 'o');
plot_rdot = 0;
if plot_rdot
  det_colour = get(h1, 'Color');
  quiver(all_dets.xy(1,:), all_dets.xy(2,:), all_dets.vxy(1,:), all_dets.vxy(2,:), ...
      'Color', det_colour);
end

% Plot belief functions over time in the global frame
all_beliefs = transform_beliefs(beliefs);
h2 = plot(all_beliefs.x, all_beliefs.y, 'x-');
plot_xydot = 0;
if plot_xydot
  belief_colour = get(h2, 'Color');
  quiver(all_beliefs.x, all_beliefs.y, all_beliefs.xdot, all_beliefs.ydot, 'Color', belief_colour);
end

legend('track true path', 'detections', 'estimated track', 'Location', 'northwest');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function transform_beliefs
%
% Transform time sequenced belief functions beliefs for plotting and visualization
%
function all_beliefs = transform_beliefs(beliefs)

for i = 1:length(beliefs)
  all_beliefs.x(i) = beliefs(i).mu(1);
  all_beliefs.xdot(i) = beliefs(i).mu(2);
  all_beliefs.y(i) = beliefs(i).mu(3);
  all_beliefs.ydot(i) = beliefs(i).mu(4);
end

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
