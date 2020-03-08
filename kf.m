%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function kf
%
% Input:
% m = current measurement {x, y, r*r_dot}
% det = current detection {r, theta, r_dot}, used for covariance R calculation
% dt = time since the last state estimate
% prev_belief = the previous belief function {mu, sig, innov, innov_cov}
% future_times = a list of times from now to predict the state
%
% Output:
% new_belief = the latest belief function {mu, sig, innov, innov_cov}
% predictions = a list future belief functions at future_times

function [new_belief, predictions] = kf(m, det, dt, prev_belief, future_times)

% Check input
assert(length(prev_belief.mu) == prod(size(prev_belief.mu)), ...
    'size(prev_belief.mu)=%d.\n', size(prev_belief.mu));
assert(size(prev_belief.sig, 1) == size(prev_belief.sig, 2));
num_state_dims = length(prev_belief.mu);
assert(length(future_times) == prod(size(future_times)));
prediction_length = length(future_times);

% Unfiltered estimate of the current state
A = kf_get_mat_A(dt);
Q = kf_get_cov_Q(dt);
mu_hat = A * prev_belief.mu;
sig_hat = A * prev_belief.sig * A' + Q;

% Kalman gain
C = kf_get_mat_C(prev_belief.mu);
R = kf_get_cov_R(det);
% fprintf('Reciprocal condition number = %d\n', rcond(C * sig_hat * C' + R));
new_belief.innov_cov = C * sig_hat * C' + R;
K = sig_hat * C' * inv(new_belief.innov_cov);

% Filtered estimate of the current state
new_belief.innov = m - C*mu_hat;
new_belief.mu = mu_hat + K * new_belief.innov;
% new_belief.sig = (eye(num_state_dims) - K * C) * sig_hat;
% covariance in Joseph form
new_belief.sig = ...
    (eye(num_state_dims) - K * C) * sig_hat * transpose(eye(num_state_dims) - K * C) ...
    + K * R * K';

% Predict future states
predictions.mu = zeros(prediction_length);
predictions.sig = zeros(prediction_length);
for i = 1:prediction_length
  t = future_times(i);
  A = kf_get_mat_A(t);
  Q = kf_get_cov_Q(t);
  predictions.mu(i) = A * new_belief.mu;
  predictions.sig(i) = A * new_belief.sig * A' + Q;
end

%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  % State model
%  %     s = [x; x_dot; y; y_dot]
%  %     s1 = A * s + N(0,Q)
%  % where N(0,Q) is the gaussian process noise with zero-mean and covariance Q.
%
%  function A = get_mat_A(dt)
%  A = [1 dt  0  0;
%       0  1  0  0;
%       0  0  1 dt;
%       0  0  0  1];
%
%  function Q = get_cov_Q(dt)
%  % Using Singer model for sampling time << acceleration/maneuvering time
%  % From Blackman Page 33 without derivation
%  % sig_m = 1; % TODO: set realistic magnitudes
%  % tau = 1;
%  % Q = 2 * sig_m^2 / tau * ...
%  %     [dt^5/20 dt^4/8 0 0;
%  %      dt^4/8  dt^3/3 0 0;
%  %      0 0 dt^5/20 dt^4/8;
%  %      0 0 dt^4/8  dt^3/3];
%
%  Q = [10 0 0 0;
%        0 1 0 0;
%        0 0 10 0;
%        0 0 0 1];
%
%  % Using constant velocity model,
%  % http://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf
%  % q = 1000;
%  % Q = q * [dt^3/3 dt^2/2   0      0;
%  %          dt^2/2   dt     0      0;
%  % 	   0       0  dt^3/3 dt^2/2;
%  % 	   0       0  dt^2/2   dt  ];
%
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  % Measurement model
%  %     m = [x; y; x*x_dot+y*y_dot ~ r*r_dot]
%  %     m = C * s + N(0,R)
%  % where N(0,R) is the gaussian measurement noise with zero-mean and covariance R.
%
%  function C = get_mat_C(s)
%  C = [   1    0    0    0;
%          0    0    1    0;
%        s(2) s(1) s(4) s(3)];
%
%  function R = get_cov_R(det)
%  r = det.r;
%  theta = det.theta;
%  r_dot = det.r_dot;
%
%  % r, r_dot, theta have stddev sig_r, sig_rdot, sig_theta, respectively
%  var = get_detection_variance(det);
%  sig_r = var.sig_r;
%  sig_rdot = var.sig_rdot;
%  sig_theta = var.sig_theta;
%
%  % Calculate measurement covariance R
%  % If x = r*cos(theta), y = r*sin(theta), then:
%  % sig_xx = sig_r^2 * cos(theta)^2 + r^2 * sig_theta^2 * sin(theta)^2;
%  % sig_xy = cos(theta) * sin(theta) * (sig_r^2 - r^2 * sig_theta^2);
%  % sig_xrrdot = r_dot * sig_r^2 * cos(theta);
%  % sig_yy = r^2 * sig_theta^2 * cos(theta)^2 + sig_r^2 * sin(theta)^2;
%  % sig_yrrdot = r_dot * sig_r^2 * sin(theta);
%  % sig_rrdot = r_dot^2 * sig_r^2 + r^2 * sig_rdot^2;
%
%  % If x = -r*sin(theta), y = r*cos(theta), then:
%  sig_xx = sig_r^2 * sin(theta)^2 + r^2 * sig_theta^2 * cos(theta)^2;
%  sig_xy = cos(theta) * sin(theta) * (-sig_r^2 + r^2 * sig_theta^2);
%  sig_xrrdot = -r_dot * sig_r^2 * sin(theta);
%  sig_yy = r^2 * sig_theta^2 * sin(theta)^2 + sig_r^2 * cos(theta)^2;
%  sig_yrrdot = r_dot * sig_r^2 * cos(theta);
%  sig_rrdot = r_dot^2 * sig_r^2 + r^2 * sig_rdot^2;
%
%  R = 0.01 * [sig_xx sig_xy sig_xrrdot;
%       sig_xy sig_yy sig_yrrdot;
%       sig_xrrdot sig_yrrdot sig_rrdot];
