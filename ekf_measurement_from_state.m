%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_measurement_from_state

function m = ekf_measurement_from_state(s)
x = s(1);
xdot = s(2);
y = s(3);
ydot = s(4);
r = sqrt(x^2+y^2);
theta = atan(-x/y);
r_dot = -xdot * sin(theta) + ydot * cos(theta);
m = [r; theta; r_dot]
end
