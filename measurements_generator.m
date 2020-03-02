%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function measurements_generator
%
% Given tracks (a vector of track structs), returns measurements at a particular time
% t. Each measurement is a struct containing {r, r_dot, theta}.
%

function meas = measurements_generator(tracks, t, radar_coords)

meas = [];
for i = 1:length(tracks);
  track = tracks(i);
  meas = [meas track2meas(track.x(t), track.y(t), track.vx(t), track.vy(t), radar_coords)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function track2meas
%
% Convert a track state {x, y, vx, vy} in the global frame to measurements in the radar
% frame. A single track state can generate one or more measurements. Each measurement is
% represented by {r, r_dot, theta} for the noisy state and {r0, r_dot0, theta0} for the
% ground truth state.
%

function meas = track2meas(x, y, vx, vy, radar_coords)

meas = [];

% Error-free measurement
new_xy = radar_coords.R' * ([x; y] - radar_coords.p);
m.r0 = sqrt(new_xy(1)^2 + new_xy(2)^2);
m.theta0 = atan(new_xy(2)/new_xy(1));
new_vxy = radar_coords.R' * ([vx; vy] - radar_coords.p);
m.r_dot0 = dot(new_vxy, [cos(m.theta0); sin(m.theta0)]);

% Add gaussian noise
% Each measurement m has a noise profile that is Gaussian in r and theta.
cov = expected_covariance;
m.r = m.r0 + cov.r*randn(1);
m.theta = m.theta0 + cov.theta*randn(1);
m.r_dot = m.r_dot0 + cov.r_dot*randn(1);

% Add extra measurement(s) for this state

% Return results
meas = [meas m];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function expectedCovariance
%
% Generate covariance for a measurement in {r, theta, r_dot}.
%

function cov = expected_covariance()
% Assume constant covariance for all measurements. In the future, can implement variable
% covariance based on a scale-factor or r.
cov.r = 2;
cov.theta = 2./180.*pi;
cov.r_dot = 1;
