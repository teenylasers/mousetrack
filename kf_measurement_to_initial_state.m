%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function kf_measurement_to_initial_state
%
% Given an input m, which can be either a measurement vector [r; theta; rdot] or a
% detection struct {r, theta, rdot}, return our best guess of the initial state vector.
%

function s = kf_measurement_to_initial_state(m)

global FLAGS

if class(m)=='struct'
  r = m.r;
  theta = m.theta;
  rdot = m.rdot;
  x = -r * sin(theta);
  y = r * cos(theta);
  xdot = -rdot * sin(theta);
  ydot = rdot * cos(theta);
else
  x = m(1);
  y = m(2);
  xdot = 0;
  ydot = 0;
end

if FLAGS.model_accel
  xdotdot = 0;
  ydotdot = 0;
  s = [x; xdot; xdotdot; y; ydot; ydotdot];
else
  s = [x; xdot; y; ydot];
end

end