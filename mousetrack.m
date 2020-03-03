% Top-level script that invokes the tracker simulator.
% All units are SI.

% Terminology
% Track = ground truth of the target's state over time
% Detection = raw data from the sensor (radar), ie {r, theta, r_dot}
% Measurement = a vector that is converted from detection, {x, y, r*r_dot}

% Set up the 2D space, i.e. the cartesian global frame.
x_size = 450;
y_size = x_size/2;
x_range.min = -x_size/2;
x_range.max = x_range.min + x_size;
y_range.min = 0;
y_range.max = y_range.min + y_size;

% Detection update frequency = 10 Hz
dt = 0.1;

% Generate tracks, described in the global frame.
% Estimate the amount of time a target takes to traverse the 2D space.
%  - average target speed: 15 m/s ~ 33.5 mph
%  - track length ~ 600 m
%  - traversal time ~ 40 s
%  - average num updates N = 400
%  - add round(randn) to slightly randomize target speed
N = 400 + round(randn(1) * 22);
tracks(1) = track_generator(x_range, y_range, N, dt);

% Set up the radar frame with respect to the global frame.
% Transformation of a point between the radar frame b and the global frame b is given by
%     b  =  p + Rb'
%     b' =  R^T(b - p)
radar_coords.p = [(x_range.max + x_range.min)/2; y_range.min];
radar_coords.bearing = pi/2;
radar_coords.R = ...
    [ cos(radar_coords.bearing) -sin(radar_coords.bearing) ; ...
      sin(radar_coords.bearing)  cos(radar_coords.bearing) ];

% Generate radar detections from a track for time indices ti
dets = [];
beta = [];
for ti = 1:1:(N-1)
  new_dets = detections_generator(tracks, ti, radar_coords);
  if length(beta) == 0

  end

  dets = [dets new_dets];
end

% Visualize the track(s) and measurements
visualize_tracks_dets(tracks, dets, radar_coords);
xlim([x_range.min-x_size*0.1 x_range.max+x_size*0.1]);
ylim([y_range.min-y_size*0.1 y_range.max+y_size*0.1]);
