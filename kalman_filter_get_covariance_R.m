%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function kalman_filter_get_covariance_R
%
% Measurement model
%     m = [x; y; x*x_dot+y*y_dot ~ r*r_dot]
%     m = C * s + N(0,R)
% where N(0,R) is the gaussian measurement noise with zero-mean and covariance R.

function R = kalman_filter_get_covariance_R(det)

if nargin<1 || det.r==nan || det.theta==nan || det.r_dot==nan
  R = [10 0 0;
       0 10 0;
       0 0 20];
  return;
end

r = det.r;
theta = det.theta;
r_dot = det.r_dot;

% r, r_dot, theta have stddev sig_r, sig_rdot, sig_theta, respectively
var = get_detection_variance(det);
sig_r = var.sig_r;
sig_rdot = var.sig_rdot;
sig_theta = var.sig_theta;

% Calculate measurement covariance R
% If x = r*cos(theta), y = r*sin(theta), then:
% sig_xx = sig_r^2 * cos(theta)^2 + r^2 * sig_theta^2 * sin(theta)^2;
% sig_xy = cos(theta) * sin(theta) * (sig_r^2 - r^2 * sig_theta^2);
% sig_xrrdot = r_dot * sig_r^2 * cos(theta);
% sig_yy = r^2 * sig_theta^2 * cos(theta)^2 + sig_r^2 * sin(theta)^2;
% sig_yrrdot = r_dot * sig_r^2 * sin(theta);
% sig_rrdot = r_dot^2 * sig_r^2 + r^2 * sig_rdot^2;

% If x = -r*sin(theta), y = r*cos(theta), then:
sig_xx = sig_r^2 * sin(theta)^2 + r^2 * sig_theta^2 * cos(theta)^2;
sig_xy = cos(theta) * sin(theta) * (-sig_r^2 + r^2 * sig_theta^2);
sig_xrrdot = -r_dot * sig_r^2 * sin(theta);
sig_yy = r^2 * sig_theta^2 * sin(theta)^2 + sig_r^2 * cos(theta)^2;
sig_yrrdot = r_dot * sig_r^2 * cos(theta);
sig_rrdot = r_dot^2 * sig_r^2 + r^2 * sig_rdot^2;

R = 0.01 * [sig_xx sig_xy sig_xrrdot;
     sig_xy sig_yy sig_yrrdot;
     sig_xrrdot sig_yrrdot sig_rrdot];
