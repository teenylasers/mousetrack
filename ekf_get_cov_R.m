%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_get_cov_R
%
% Measurement model
%     m = [r; theta; r_dot]
%     m = c(s) + N(0,R)
% where N(0,R) is the gaussian measurement noise with zero-mean and covariance R.

function R = ekf_get_cov_R(det)

% r, r_dot, theta have stddev sig_r, sig_rdot, sig_theta, respectively
var = get_detection_variance(det);
sig_r = var.sig_r;
sig_rdot = var.sig_rdot;
sig_theta = var.sig_theta;

R = [sig_r^2 0 0;
  0 sig_theta^2 0;
  0 0 sig_rdot^2];