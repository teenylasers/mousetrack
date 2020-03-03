%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function kalman_filter
%
% Input:
% m = current measurement {x, y, r*r_dot}
% det = current detection {r, theta, r_dot}, used for covariance R calculation
% dt = time since the last state estimate
% prev_beta = the previous belief function, characterized by the attributes {mu, sig}
% future_times = a list of times from now to predict the state
%
% Output:
% new_beta = the latest belief function, characterized by {mu, sig}
% predictions = a list future belief functions at future_times

function new_beta, predictions = kalman_filter(m, det, dt, prev_beta, future_times)

% Check input
assert(length(prev_beta.mu) == prod(size(prev_beta.mu)));
assert(size(prev_beta.sig, 1) == size(prev_beta.sig, 2));
num_state_dims = length(prev_beta.mu);
assert(length(future_times) == prod(size(future_times)));
prediction_length = length(future_times);

% Unfiltered estimate of the current state
A = get_matrix_A(dt);
Q = get_covariance_Q(dt);
mu_hat = A * prev_beta.mu;
sig_hat = A * prev_beta.sig * A' + Q;

% Kalman gain
C = get_matrix_C(prev_beta.mu);
R = get_covariance_Q(det.r, det.theta, det.r_dot);
% TODO: is inv() a good idea? numerical stability?
K = sig_hat * C' * inv(C * sig_hat * C' + R);

% Filtered estimate of the current state
m_residual = m - C*mu_hat;
new_beta.mu = mu_hat + K * m_residual;
new_beta.sig = (eye(num_state_dims) - K * C) * sig_hat;

% Predict future states
predictions.mu = zeros(prediction_length);
predictions.sig = zeros(prediction_length);
for i = 1:prediction_length
  t = futre_times[i];
  A = get_matrix_A(t);
  Q = get_covariance_Q(t);
  predictions.mu[i] = A * new_beta.mu;
  predictions.sig[i] = A * new_beta.sig * A' + Q;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State model
%     s = [x; x_dot; y; y_dot]
%     s1 = A * s + N(0,Q)
% where N(0,Q) is the gaussian process noise with zero-mean and covariance Q.

function A = get_matrix_A(dt)
A = [1 dt  0  0;
     0  1  0  0;
     0  0  1 dt;
     0  0  0  1];

function Q = get_covariance_Q(dt)
% Using Singer model for sampling time << acceleration/maneuvering time
% From Blackman Page 33 without derivation
sig_m = 1; % TODO: set realistic magnitudes
tau = 1;
Q = 2 * sig_m^2 / tau * ...
    [dt^5/20 dt^4/8 0 0;
     dt^4/8  dt^3/3 0 0;
     0 0 dt^5/20 dt^4/8;
     0 0 dt^4/8  dt^3/3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measurement model
%     m = [x; y; x*x_dot+y*y_dot]
%     m = C * s + N(0,R)
% where N(0,R) is the gaussian measurement noise with zero-mean and covariance R.

function C = get_matrix_C(s)
C = [   1    0    0    0;
        0    0    1    0;
      s(2) s(1) s(4) s(3)];

function R = get_covariance_R(r, theta, r_dot)
% r, r_dot, theta have stddev sig_r, sig_rdot, sig_theta, respectively
sig_r = 1; % TODO: set realistic magnitudes
sig_rdot = 1;
sig_theta = 1;

sig_xx = sig_r^2 * cos(theta)^2 + r^2 * sig_theta^2 * sin(theta)^2;
sig_xy = cos(theta) * sin(theta) * (sig_r^2 - r^2 * sig_theta^2);
sig_xrrdot = r_dot * sig_r^2 * cos(theta);
sig_yy = r^2 * sig_theta^2 * cos(theta)^2 + sig_r^2 * sin(theta)^2;
sig_yrrdot = r_dot * sig_r^2 * sin(theta);
sig_rrdot = r_dot^2 * sig_r^2 + r^2 * sig_rdot^2;

R = [sig_xx sig_xy sig_xrrdot;
     sig_xy sig_yy sig_yrrdot;
     sig_xrrdot sig_yrrdot sig_rrdot];
