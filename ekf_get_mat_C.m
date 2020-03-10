%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_get_mat_C
%
% Measurement model
%     m = [r; theta; rdot]
%     m = c(s) + N(0,R)
% where N(0,R) is the gaussian measurement noise with zero-mean and covariance R.

function C = ekf_get_mat_C(s)

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

if FLAGS.model_accel
  C = [x/sqrt(x^2+y^2)    0    0     y/sqrt(x^2+y^2)     0    0;
       -1/(1+x^2/y^2)/y   0    0    x/(1+x^2/y^2)/y^2    0    0;
       -x^2*xdot/(1+x^2/y^2)^(3/2)/y^3 + xdot/sqrt(1+x^2/y^2)/y - x*ydot/(1+x^2/y^2)^(3/2)/y^2 ...
       x/sqrt(1+x^2/y^2)/y ...
       0 ...
       x^3*xdot/(1+x^2/y^2)^(3/2)/y^4 - x*xdot/sqrt(1+x^2/y^2)/y^2 + x^2*ydot/(1+x^2/y^2)^(3/2)/y^3 ...
       1/sqrt(1+x^2/y^2) ...
       0 ];
else
  C = [x/sqrt(x^2+y^2)    0     y/sqrt(x^2+y^2)     0;
       -1/(1+x^2/y^2)/y   0    x/(1+x^2/y^2)/y^2    0;
       -x^2*xdot/(1+x^2/y^2)^(3/2)/y^3 + xdot/sqrt(1+x^2/y^2)/y - x*ydot/(1+x^2/y^2)^(3/2)/y^2 ...
       x/sqrt(1+x^2/y^2)/y ...
       x^3*xdot/(1+x^2/y^2)^(3/2)/y^4 - x*xdot/sqrt(1+x^2/y^2)/y^2 + x^2*ydot/(1+x^2/y^2)^(3/2)/y^3 ...
       1/sqrt(1+x^2/y^2)];
end

end % function ekf_get_mat_C
