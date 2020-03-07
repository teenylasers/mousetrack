%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function initiate_tracks
%
% Returns a belief function belief that represents a new track.
%
% Input:
% dets = an array of detections {r, theta, r_dot}, the last element is the latest
% detection

function belief = initiate_track(dets)

m = convert_detection_to_measurement(dets(end));
xdot = -dets(end).r_dot * sin(dets(end).theta);
ydot = dets(end).r_dot * cos(dets(end).theta);
belief.mu = [m(1); xdot; m(2); ydot];
belief.sig = eye(4) * 1e9;
belief.innov = [0; 0; 0];
belief.innov_cov = eye(length(belief.innov));

% Decide whether there has been enough detections to initiate a track.
% if length(dets) < 2
%   can_init_track = 0;
% else
%   can_init_track = 1;
% end
%
% % If cannot initiate a track, then return an all-zero belief
% if ~can_init_track
%   belief.mu = zeros(4,1);
%   belief.sig = zeros(4);
%   belief.innov = zeros(3,1);
%   belief.innov_cov = zeros(3);
% else
%   prev_meas = convert_detection_to_measurement(dets(length(dets)-1));
%   curr_meas = convert_detection_to_measurement(dets(end));
%   global dt
%   xdot = (curr_meas(1)-prev_meas(1))/dt;
%   ydot = (curr_meas(2)-prev_meas(2))/dt;
%   belief.mu = [curr_meas(1); xdot; curr_meas(2); ydot];
%   belief.sig = [10  0  0  0;
%                  0  0.1  0  0;
% 		 0  0  10  0;
% 		 0  0  0  0.1]; % TODO: not correct
%   belief.innov = [0; 0; 0];
%   belief.innov_cov = zeros(3);
% end
