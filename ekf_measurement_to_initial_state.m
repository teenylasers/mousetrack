%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_measurement_to_initial_state
%
% Given an input m, which can be either a measurement vector [r; theta; rdot] or a
% detection struct {r, theta, rdot}, return our best guess of the initial state vector.
%

function s = ekf_measurement_to_initial_state(m)

global FLAGS

if class(m)=='struct'
  r = m.r;
  theta = m.theta;
  rdot = m.rdot;
else
  r = m(1);
  theta = m(2);
  rdot = m(3);
end

x = -r * sin(theta);
y = r * cos(theta);
xdot = -rdot * sin(theta);
ydot = rdot * cos(theta);

if FLAGS.model_accel
  xdotdot = 0;
  ydotdot = 0;
  s = [x; xdot; xdotdot; y; ydot; ydotdot];
else
  s = [x; xdot; y; ydot];
end

end