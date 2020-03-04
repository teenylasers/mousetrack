%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization: overlay detections in the radar frame on track in the global frame
function visualize_tracks_predictions(tracks, beliefs, predictions, new_fig, hold_on)

if new_fig
  figure; hold on;
end

% Plot the tracks in the global frame
for i = 1:length(tracks)
  tr = tracks(i);
  plot(tr.x, tr.y, '.');
  % quiver(tr.x, tr.y, tr.vx, tr.vy);
end

% Plot belief functions over time in the global frame
all_beliefs = transform_beliefs(beliefs);
h = plot(all_beliefs.x, all_beliefs.y, 'x');
belief_colour = get(h, 'Color');
quiver(all_beliefs.x, all_beliefs.y, all_beliefs.xdot, all_beliefs.ydot, 'Color', belief_colour);

%legend('track true path', 'track estimation');
if ~hold_on
  hold off;
end

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
