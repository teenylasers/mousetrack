%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function track_generator
%
% Generates a track, returns a vector (length N) that represents points along the track.
%
% x_range, y_range:
% The extent of the 2D space that the track can occupy. x_range and y_range are each a
% struct that contains a min and a max field.
%
% N:
% Number of points to represent the track.
%
% track:
% Each point along the track is described by {x, y, vx, vy}
%

function track = track_generator(x_range, y_range, N)

% Generate a set of way_points that defines the track
way_points = way_points_generator(x_range, y_range);

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

% Calculate velocity (vx, vy). Assume time between adjacent track points is unit 1.
vx = zeros(1,N);
vy = zeros(1,N);
vx(1) = xx(2)-xx(1);       % starting velocity
vy(1) = yy(2)-yy(1);
vx(N) = xx(N)-xx(N-1);     % ending velocity
vy(N) = yy(N)-yy(N-1);
for i=2:N-1
  % intermediate velocities are the average with respect to points before and after
  vx(i) = ((xx(i)-xx(i-1)) + (xx(i+1)-xx(i)))/2;
  vy(i) = ((yy(i)-yy(i-1)) + (yy(i+1)-yy(i)))/2;
end

track.x = xx;
track.y = yy;
track.vx = vx;
track.vy = vy;
track.wpts = way_points;

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
% x_range, y_range:
% The extent of the 2D space that the track can occupy. x_range and y_range are each a
% struct that contains a min and a max field.
%

function way_points = way_points_generator(x_range, y_range)

% Sanity check input
assert(x_range.min < x_range.max);
assert(y_range.min < y_range.max);

% Use a helper function to generate way-points
way_points = way_points_generator_monotonic_x(x_range, y_range);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function way_points_generator_monotonic_x
%
% Helper function to way_points_generator(). way-points will have monotonically increasing
% x, y may change in any direction.

function way_points = way_points_generator_monotonic_x(x_range, y_range)

% Generate M way_points, M is a random number between 2 and M_max.
M_max = 8;
M = randi([2, M_max], 1);
x = zeros(1, M);
y = zeros(1, M);

% Generate the first way-point, i.e. track starting point.
% Track always starts from the left-end, thus x(1) = x_range.min; however, it can be
% anywhere between y_range.min and max.
x(1) = x_range.min;
y(1) = y_range.min + (y_range.max - y_range.min) * rand(1);

% Consecutive way-points will never vary for more than delta_x_bound and delta_y_bound.
delta_x_bound = (x_range.max - x_range.min) / (M-1);
delta_y_bound = (y_range.max - y_range.min) / M_max;

for i = 2:M
  % x
  %delta_x = rand(1) * delta_x_bound;
  delta_x = delta_x_bound;
  assert(x(i-1) + delta_x <= x_range.max, ...
      'x(i-1) = %f, delta_x = %f, x_range.max = %f', ...
      x(i-1), delta_x, x_range.max);
  x(i) = x(i-1) + delta_x;
  % y
  delta_y = randn(1) * delta_y_bound;
  if y(i-1) + delta_y < y_range.min || y(i-1) + delta_y > y_range.max
    y(i) = y(i-1) - delta_y;
  else
    y(i) = y(i-1) + delta_y;
  end
end

way_points.x = x;
way_points.y = y;