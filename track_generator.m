%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function track_generator
%
% Generates a random smooth track, using spline with randomly generated waypoints. Returns
% a vector (length N) that represents points along the track.
%
% x_axis, y_axis:
% The extent of the 2D space that the track can occupy. x_axis and y_axis are each a
% struct that contains a min and a max field.
%
% N:
% Number of points to represent the track.
%
% dt:
% Time between successive track points.
%
% track:
% Each point along the track is described by {x, y, vx, vy}
%

function track = track_generator(x_axis, y_axis, N, dt)

% Generate a set of way_points that defines the track
way_points = way_points_generator(x_axis, y_axis);

% Check that way_points.x and way_points.y are 1D vectors
assert(prod(size(way_points.x)) == prod(length(way_points.x)), ...
    'way_points.x is not a 1D vector.');
assert(prod(size(way_points.y)) == prod(length(way_points.y)), ...
    'way_points.y is not a 1D vector.');

% Check that way_points.x and way_points.y have the same length, and length >= 2
assert(length(way_points.x) == length(way_points.y), ...
    'way_points.x and y have different lengths.');
assert(length(way_points.x) >= 2, 'way_points needs to have 2 or more points.');

% Use spline to generate a track through way_points
num_way_points = length(way_points.x);
x_start = way_points.x(1);
x_end = way_points.x(num_way_points);
xx = linspace(x_start, x_end, N);
yy = spline(way_points.x, way_points.y, xx);

% Calculate velocity (vx, vy).
vx = zeros(1,N);
vy = zeros(1,N);
vx(1) = (xx(2)-xx(1)) / dt;      % starting velocity
vy(1) = (yy(2)-yy(1)) / dt;
vx(N) = (xx(N)-xx(N-1)) / dt;    % ending velocity
vy(N) = (yy(N)-yy(N-1)) / dt;
for i=2:N-1
  % intermediate velocities are the average with respect to points before and after
  vx(i) = ((xx(i)-xx(i-1)) + (xx(i+1)-xx(i)))/dt/2;
  vy(i) = ((yy(i)-yy(i-1)) + (yy(i+1)-yy(i)))/dt/2;
end

track.x = xx;
track.y = yy;
track.vx = vx;
track.vy = vy;
track.wpts = way_points;

end % function track_generator

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function way_points_generator
%
% Generates a set of way-points that will define a track.
%
% way_points:
% A struct of x and y coordinates, way_points.x and way_points.y should have the same
% length. The first and last terms of way_points represent the start and end points of the
% track
%
% x_axis, y_axis:
% The extent of the 2D space that the track can occupy. x_axis and y_axis are each a
% struct that contains a min and a max field.
%

function way_points = way_points_generator(x_axis, y_axis)

% Sanity check input
assert(x_axis.min < x_axis.max);
assert(y_axis.min < y_axis.max);

% Use a helper function to generate way-points
way_points = way_points_generator_monotonic_x(x_axis, y_axis);

end % function way_points_generator

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function way_points_generator_monotonic_x
%
% Helper function to way_points_generator(). way-points will have monotonically increasing
% x, y may change in any direction.


function way_points = way_points_generator_monotonic_x(x_axis, y_axis)

% Generate M way_points, M is a random number between 2 and M_max.
M_max = 8;
M = randi([2, M_max], 1);
x = zeros(1, M);
y = zeros(1, M);

% Generate the first way-point, i.e. track starting point.
% Track always starts from the left-end, thus x(1) = x_axis.min; however, it can be
% anywhere between y_axis.min and max.
x(1) = x_axis.min;
y(1) = y_axis.min + y_axis.extent * rand(1);

% Consecutive way-points will never vary for more than delta_x_bound and delta_y_bound.
delta_x_bound = x_axis.extent / (M-1);
delta_y_bound = y_axis.extent / M_max;

for i = 2:M
  % x
  %delta_x = rand(1) * delta_x_bound;
  delta_x = delta_x_bound;
  assert(x(i-1) + delta_x <= x_axis.max, ...
      'x(i-1) = %f, delta_x = %f, x_axis.max = %f', ...
      x(i-1), delta_x, x_axis.max);
  x(i) = x(i-1) + delta_x;
  % y
  delta_y = randn(1) * delta_y_bound;
  if y(i-1) + delta_y < y_axis.min || y(i-1) + delta_y > y_axis.max
    y(i) = y(i-1) - delta_y;
  else
    y(i) = y(i-1) + delta_y;
  end
end

way_points.x = x;
way_points.y = y;

end % function way_points_generator_monotonic_x
