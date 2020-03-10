%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization: overlay detections in the radar frame on track in the global frame
function visualize_tracker_results(FLAGS, radar_coords, tracks, dets, meas, beliefs)

figure; hold on;

% Set colors
det_colour = [0.9290 0.6940 0.1250];
truth_colour = [0 0.4470 0.7410];
belief_colour = [0.8500 0.3250 0.0980];

% Plot detections over time in the global frame
if ~isempty(dets)
  all_dets = transform_detections(dets, radar_coords);
  h = polar(all_dets.theta + radar_coords.bearing, all_dets.r, 'o');
  h.Color = det_colour;
  plot_rdot = 0;
  if plot_rdot
    quiver(all_dets.xy(1,:), all_dets.xy(2,:), all_dets.vxy(1,:), all_dets.vxy(2,:), ...
	'Color', det_colour);
  end
end

% Plot measurements over time in the global frame
if isempty(dets) && ~isempty(meas)
  if FLAGS.run_kf || FLAGS.debug_kf
    plot(meas(1,:), meas(2,:), 'o', 'Color', det_colour);
  elseif FLAGS.run_ekf || FLAGS.debug_ekf
    plot(-meas(1,:).*sin(meas(2,:)), meas(1,:).*cos(meas(2,:)), 'o', 'Color', det_colour);
  end
end

% Plot the tracks in the global frame
for i = 1:length(tracks)
  tr = tracks(i);
  plot(tr.x, tr.y, '.', 'Color', truth_colour);
  % quiver(tr.x, tr.y, tr.vx, tr.vy, 'Color', truth_colour);
end

% Plot belief functions over time in the global frame
all_beliefs = transform_beliefs(beliefs);
plot(all_beliefs.x, all_beliefs.y, '-', 'LineWidth', 2, 'Color', belief_colour);
plot_xydot = 1;
if plot_xydot
  quiver(all_beliefs.x, all_beliefs.y, all_beliefs.xdot, all_beliefs.ydot, 'Color', belief_colour);
end

legend('dets/meas', 'track true path', 'estimated track', 'Location', 'northwest');
global x_axis
global y_axis
%xlim([x_axis.min-x_axis.extent*0.1 x_axis.max+x_axis.extent*0.1]);
%ylim([y_axis.min-y_axis.extent*0.1 y_axis.max+y_axis.extent*0.1]);
hold off;

end % end function visualize_tracker_results

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

end % end function transform_beliefs

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function transform_detections
%
% Transform time sequenced detections for plotting and visualization
%

function all_dets = transform_detections(dets, radar_coords)

for i = 1:length(dets)
  all_dets.r(i) = dets(i).r;
  all_dets.rdot(i) = dets(i).rdot;
  all_dets.theta(i) = dets(i).theta;
  all_dets.xy(:,i) = radar_coords.p + ...
      dets(i).r * radar_coords.R * [cos(dets(i).theta); sin(dets(i).theta)];
  all_dets.vxy(:,i) = radar_coords.p + ...
      dets(i).rdot * radar_coords.R * [cos(dets(i).theta); sin(dets(i).theta)];

  all_dets.r0(i) = dets(i).r0;
  all_dets.rdot0(i) = dets(i).rdot0;
  all_dets.theta0(i) = dets(i).theta0;
  all_dets.xy0(:,i) = radar_coords.p + ...
      dets(i).r0 * radar_coords.R * [cos(dets(i).theta0); sin(dets(i).theta0)];
  all_dets.vxy0(:,i) = radar_coords.p + ...
      dets(i).rdot0 * radar_coords.R * [cos(dets(i).theta0); sin(dets(i).theta0)];
end

end % end functon transform_detections
