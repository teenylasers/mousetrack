%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function kalman_filter_get_matrix_A
%
% State model
%     s = [x; x_dot; y; y_dot]
%     s1 = A * s + N(0,Q)
% where N(0,Q) is the gaussian process noise with zero-mean and covariance Q.

function A = kalman_filter_get_matrix_A(dt)
A = [1 dt  0  0;
     0  1  0  0;
     0  0  1 dt;
     0  0  0  1];