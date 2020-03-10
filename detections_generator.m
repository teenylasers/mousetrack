%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function detections_generator
%
% Given tracks (a vector of track structs), returns detections at a particular time
% index ti. Each detection is a struct containing {r, rdot, theta}.
%

function det = detections_generator(tracks, ti, radar_coords)

det = [];
for i = 1:length(tracks);
  track = tracks(i);
  det = [det ...
	track2det(track.x(ti), track.y(ti), track.vx(ti), track.vy(ti), radar_coords)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function track2det
%
% Convert a track state {x, y, vx, vy} in the global frame to detections in the radar
% frame. A single track state can generate one or more detections. Each detection is
% represented by {r, rdot, theta} for the noisy state and {r0, rdot0, theta0} for the
% ground truth state.
%

function det = track2det(x, y, vx, vy, radar_coords)

det = [];

% Error-free detection
new_xy = radar_coords.R' * ([x; y] - radar_coords.p);
d.r0 = sqrt(new_xy(1)^2 + new_xy(2)^2);
d.theta0 = atan(new_xy(2)/new_xy(1));
new_vxy = radar_coords.R' * ([vx; vy] - radar_coords.p);
d.rdot0 = dot(new_vxy, [cos(d.theta0); sin(d.theta0)]);

% Add gaussian noise
% Each detection m has a noise profile that is Gaussian in r and theta.
cov = expected_covariance;
d.r = d.r0 + cov.r*randn(1);
d.theta = d.theta0 + cov.theta*randn(1);
d.rdot = d.rdot0 + cov.rdot*randn(1);

% Add extra detection(s) for this state to simulate multiple returns per target, or
% clutter and false positives

% Return results
det = [det d];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function expectedCovariance
%
% Generate covariance for a detection in {r, theta, rdot}.
%

function cov = expected_covariance()
% Assume constant covariance for all detections. In the future, can implement variable
% covariance based on a scale-factor or r.
cov.r = 2;
cov.theta = 2./180.*pi;
cov.rdot = 0.1;
