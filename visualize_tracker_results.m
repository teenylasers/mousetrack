%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization: overlay detections in the radar frame on track in the global frame
function visualize_tracker_results(radar_coords, tracks, dets, meas, beliefs)

global FLAGS

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
  tr = tracks{i};
  plot(tr.x, tr.y, '.', 'Color', truth_colour);
  % quiver(tr.x, tr.y, tr.vx, tr.vy, 'Color', truth_colour);
end

% Plot all belief functions over time in the global frame
if FLAGS.run_kf || FLAGS.run_ekf || FLAGS.debug_kf || FLAGS.debug_ekf
  all_beliefs = transform_beliefs_kf(beliefs);
  plot(all_beliefs.x, all_beliefs.y, '-', 'LineWidth', 2, 'Color', belief_colour);
  plot_xydot = 1;
  if plot_xydot
    quiver(all_beliefs.x, all_beliefs.y, all_beliefs.xdot, all_beliefs.ydot, 'Color', belief_colour);
  end
end

% Plot all belief function probability distribution over time in the global frame.
if FLAGS.run_phd_gmm
  pdfs = transform_beliefs_phd(beliefs);
  for t = 1:length(pdfs)
    pt = pdfs(t);
    for i = 1:length(pt)
      visualize_gaussian(pt(i).mu, pt(i).sig, pt(i).w);
    end
  end
end

legend('dets/meas', 'track true path', 'estimated track', 'Location', 'northwest');
global x_axis
global y_axis
%xlim([x_axis.min-x_axis.extent*0.1 x_axis.max+x_axis.extent*0.1]);
%ylim([y_axis.min-y_axis.extent*0.1 y_axis.max+y_axis.extent*0.1]);
hold off;

end % end function visualize_tracker_results

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function transform_beliefs_phd
%
% Transform time sequenced belief functions beliefs to x-y gaussians for plotting. beliefs
% is a cell array, where each cell is a list of belief functions.

function pdfs = transform_beliefs_phd(beliefs)

global FLAGS

% For now, all filters describe beliefs as gaussians
assert(FLAGS.run_phd_gmm==1, 'Only handles FLAGS.run_phd_gmm for now.');

pdfs = {};
for t = 1:length(beliefs)
  bt = beliefs{t};
  gs = [];
  for i = 1:length(bt)
    if FLAGS.model_accel
      g.mu = [bt{i}.mu(1); bt{i}.mu(4)];
      g.sig = [bt{i}.sig(1); bt{i}.sig(4)];
    else
      g.mu = [bt{i}.mu(1); bt{i}.mu(3)];
      g.sig = [bt{i}.sig(1); bt{i}.sig(3)];
    end
    g.w = bt{i}.w;
    gs(end+1) = g;
  end
  pdfs{end+1} = gs;
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function transform_beliefs
%
% Transform time sequenced belief functions beliefs for plotting and visualization. beliefs
% is a list of belief functions.

function all_beliefs = transform_beliefs_kf(beliefs)
global FLAGS
for t = 1:length(beliefs)
  bt = beliefs{t};
  for i = 1:length(bt)
    if FLAGS.model_accel
      all_beliefs.x(t) = bt{i}.mu(1);
      all_beliefs.xdot(t) = bt{i}.mu(2);
      all_beliefs.y(t) = bt{i}.mu(4);
      all_beliefs.ydot(t) = bt{i}.mu(5);
    else
      all_beliefs.x(t) = bt{i}.mu(1);
      all_beliefs.xdot(t) = bt{i}.mu(2);
      all_beliefs.y(t) = bt{i}.mu(3);
      all_beliefs.ydot(t) = bt{i}.mu(4);
    end
  end
end
end % end function transform_beliefs

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function transform_detections
%
% Transform time sequenced detections for plotting and visualization
%

function all_dets = transform_detections(dets, radar_coords)

all_dets.r = [];
all_dets.rdot = [];
all_dets.theta = [];
all_dets.xy = [];
all_dets.vxy = [];
all_dets.r0 = [];
all_dets.rdot0 = [];
all_dets.theta0 = [];
all_dets.xy0 = [];
all_dets.vxy0 = [];

for t = 1:length(dets)
  dets_at_t = dets{t};
  for i = 1:length(dets_at_t);
    all_dets.r(end+1) = dets_at_t{i}.r;
    all_dets.rdot(end+1) = dets_at_t{i}.rdot;
    all_dets.theta(end+1) = dets_at_t{i}.theta;
    all_dets.xy(:,end+1) = radar_coords.p + ...
        dets_at_t{i}.r * radar_coords.R * [cos(dets_at_t{i}.theta); sin(dets_at_t{i}.theta)];
    all_dets.vxy(:,end+1) = radar_coords.p + ...
        dets_at_t{i}.rdot * radar_coords.R * [cos(dets_at_t{i}.theta); sin(dets_at_t{i}.theta)];

    all_dets.r0(end+1) = dets_at_t{i}.r0;
    all_dets.rdot0(end+1) = dets_at_t{i}.rdot0;
    all_dets.theta0(end+1) = dets_at_t{i}.theta0;
    all_dets.xy0(:,end+1) = radar_coords.p + ...
        dets_at_t{i}.r0 * radar_coords.R * [cos(dets_at_t{i}.theta0); sin(dets_at_t{i}.theta0)];
    all_dets.vxy0(:,end+1) = radar_coords.p + ...
        dets_at_t{i}.rdot0 * radar_coords.R * [cos(dets_at_t{i}.theta0); sin(dets_at_t{i}.theta0)];
  end
end

end % end functon transform_detections
