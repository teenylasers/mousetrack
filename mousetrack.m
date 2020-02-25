% Top-level script that invokes the tracker simulator

% Set up the 2D space, i.e. the cartesian global frame.
x_size = 6000;
y_size = x_size/2;
x_range.min = -x_size/2;
x_range.max = x_range.min + x_size;
y_range.min = 0;
y_range.max = y_range.min + y_size;

% Generate tracks, described in the global frame.
N = 100;
tracks(1) = track_generator(x_range, y_range, N);

% Set up the radar frame with respect to the global frame.
% Transformation of a point between the radar frame b and the global frame b is given by
%     b  =  p + Rb'
%     b' =  R^T(b - p)
radar_coords.p = [(x_range.max + x_range.min)/2; y_range.min];
radar_coords.bearing = pi/2;
radar_coords.R = ...
    [ cos(radar_coords.bearing) -sin(radar_coords.bearing) ; ...
      sin(radar_coords.bearing)  cos(radar_coords.bearing) ];

% Generate radar measurements from a track for times t
meas = [];
for t = 1:1:(N-1)
  meas = [meas measurements_generator(tracks, t, radar_coords)];
end

visualize_tracks_meas(tracks, meas, radar_coords);
