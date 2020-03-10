%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_get_mat_A
%
% State model
%     s = [x; xdot; y; ydot]
%     s1 = A * s + N(0,Q)
% where N(0,Q) is the gaussian process noise with zero-mean and covariance Q.

function A = ekf_get_mat_A(dt)

global FLAGS

if FLAGS.model_accel
  A = [1 dt dt^2/2 0 0 0;
       0  1   dt   0 0 0;
       0  0    1   0 0 0;
       0 0 0 1 dt dt^2/2;
       0 0 0 0  1   dt  ;
       0 0 0 0  0    1  ];
else
  A = [1 dt  0  0;
       0  1  0  0;
       0  0  1 dt;
       0  0  0  1];
end

end % function ekf_get_mat_A
