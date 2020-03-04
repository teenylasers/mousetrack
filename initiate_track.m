%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function initiate_tracks
%
% Returns a belief function belief that represents a new track.
%
% Input:
% m = measurement vector {x, y, r*r_dot}
% det = detection struct {r, theta, r_dot}

function belief = initiate_track(m, det)
% Naive track initiation for now
xdot = 0; % TODO: not correct
ydot = 0; % TODO: not correct
belief.mu = [m(1); xdot; m(2); ydot];
belief.sig = [10  0   0  0;
             0  1   0  0;
	     0  0  10  0;
	     0  0   0  1]; % TODO: not correct
