%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function track_generator_ekf_debug
%
% Generates a track, returns a vector (length N) that represents points along the track.
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

function track = track_generator_ekf_debug(x_axis, y_axis, N, dt)

x = zeros(1,N);
y = zeros(1,N);
vx = zeros(1,N);
vy = zeros(1,N);
A = ekf_get_mat_A(dt);
Q = ekf_get_cov_Q(dt);

% Track always starts from the left-end, thus x(1) = x_axis.min; however, it can be
% anywhere between y_axis.min and max.
x(1) = x_axis.min;
y(1) = y_axis.min + y_axis.extent * rand(1);
vx(1) = x_axis.extent/N/dt;
vy(1) = y_axis.extent/round(2.2*N)/dt;

% Iterate s = As+N(0,Q) to N-point track. If track exceeds the domain set by x_axis and
% y_axis, then return.
for i = 2:N
  s = A * [x(i-1); vx(i-1); y(i-1); vy(i-1)];
  %+ get_random_gaussian_distr_vectors(zeros(size(A,1),1), Q, 1)
  x(i) = s(1);
  vx(i) = s(2);
  y(i) = s(3);
  vy(i) = s(4);
end

track.x = x;
track.y = y;
track.vx = vx;
track.vy = vy;
