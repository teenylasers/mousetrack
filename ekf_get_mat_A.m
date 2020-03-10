%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_get_mat_A
%
% State model
%     s = [x; xdot; y; ydot]
%     s1 = A * s + N(0,Q)
% where N(0,Q) is the gaussian process noise with zero-mean and covariance Q.

function A = ekf_get_mat_A(dt)
A = [1 dt  0  0;
     0  1  0  0;
     0  0  1 dt;
     0  0  0  1];
