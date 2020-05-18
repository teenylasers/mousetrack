%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function track_generator_ekf_debug
%
% Generates a track, using (e)kf_get_mat_A/cov_Q, for debug purposes, so that the (e)kf
% filter should exactly match the track. Returns a vector (length N) that represents points
% along the track.
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

function track = track_generator_debug(x_axis, y_axis, N, dt)

global FLAGS

x = zeros(1,N);
y = zeros(1,N);
vx = zeros(1,N);
vy = zeros(1,N);
if FLAGS.debug_kf && ~FLAGS.debug_ekf
  A = kf_get_mat_A(dt);
  Q = kf_get_cov_Q(dt);
elseif ~FLAGS.debug_kf && FLAGS.debug_ekf
  A = ekf_get_mat_A(dt);
  Q = ekf_get_cov_Q(dt);
else
  fprintf(['Unknown FLAGS setting: ', FLAGS, '\n']);
end

% Track always starts from the left-end, thus x(1) = x_axis.min; however, it can be
% anywhere between y_axis.min and max.
x(1) = x_axis.min;
y(1) = y_axis.min + y_axis.extent * rand(1);
vx(1) = x_axis.extent/N/dt;
if x(1) > x_axis.min + x_axis.extent/2
  vy(1) = y_axis.extent/round(2.2*N)/dt * -1;
else
  vy(1) = y_axis.extent/round(2.2*N)/dt;
end
if FLAGS.model_accel
  ax = zeros(1,N);
  ay = zeros(1,N);
  ax(1) = 0;
  ay(1) = 0;
end
% Iterate s = As+N(0,Q) to N-point track. If track exceeds the domain set by x_axis and
% y_axis, then return.
for i = 2:N
  if FLAGS.model_accel
    s = A * [x(i-1); vx(i-1); ax(i-1); y(i-1); vy(i-1); ay(i-1)];
    %+ get_random_gaussian_distr_vectors(zeros(size(A,1),1), Q, 1)
    x(i) = s(1);
    vx(i) = s(2);
    ax(i) = s(3);
    y(i) = s(4);
    vy(i) = s(5);
    ay(i) = s(6);
  else
    s = A * [x(i-1); vx(i-1); y(i-1); vy(i-1)] ...
        + get_random_gaussian_distr_vectors(zeros(size(A,1),1), Q, 1);
    x(i) = s(1);
    vx(i) = s(2);
    y(i) = s(3);
    vy(i) = s(4);
  end
end

track.x = x;
track.y = y;
track.vx = vx;
track.vy = vy;
if FLAGS.model_accel
  track.ax = ax;
  track.ay = ay;
end

end % function track_generator_ekf_debug
