% Top-level script that invokes the tracker simulator

% Set up the 2D space, i.e. the cartesian global frame.
x_size = 6000;
y_size = x_size/2;
x_range.min = -x_size/2;
x_range.max = x_range.min + x_size;
y_range.min = 0;
y_range.max = y_range.min + y_size;

% Generate a track. Tracks are described in the global frame.
N = 100;
[track, wpts] = track_generator(x_range, y_range, N);

% Visualize the track
figure; hold on;
plot(wpts.x, wpts.y, 'o', track.x, track.y, 'x');
quiver(track.x, track.y, track.vx, track.vy);
xlim([x_range.min x_range.max]); ylim([y_range.min y_range.max]);
hold off;

% Set up the radar frame with respect to the global frame.
% Transformation of a point between the radar frame b and the global frame b is given by
%     b  =  p + Rb'
%     b' =  R^T(b - p)
radar_coords.p = [(x_range.max - x_range.min)/2  y_range.min];
radar_bearing = pi/2;
radar_coords.R = ...
    [ cos(radar_bearing)  sin(radar_bearing); ...
     -sin(radar_bearing)  cos(radar_bearing)];

% Generate radar measurements from a track
% meas = measurements_generator(track, radar_coords);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tests:
%
% Change the 2D space in global coords, i.e. change x_range.min and y_range.min
%
% Change radar location and bearing.
