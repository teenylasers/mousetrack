%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_measurement_from_state

function m = ekf_measurement_from_state(s)
global FLAGS
if FLAGS.model_accel
  x = s(1);
  xdot = s(2);
  y = s(4);
  ydot = s(5);
else
  x = s(1);
  xdot = s(2);
  y = s(3);
  ydot = s(4);
end
r = sqrt(x^2+y^2);
theta = atan(-x/y);
rdot = -xdot * sin(theta) + ydot * cos(theta);
m = [r; theta; rdot];
end
