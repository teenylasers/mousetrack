%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf
%
% Input:
% m = current measurement {x, y, r*rdot}
% det = current detection {r, theta, rdot}, used for covariance R calculation
% dt = time since the last state estimate
% prev_belief = the previous belief function {mu, sig, innov, innov_cov}
% future_times = a list of times from now to predict the state
%
% Output:
% new_belief = the latest belief function {mu, sig, innov, innov_cov}
% predictions = a list future belief functions at future_times

function [new_belief, predictions] = ekf(m, det, dt, prev_belief, future_times)

% Check input
assert(length(prev_belief.mu) == prod(size(prev_belief.mu)), ...
    'size(prev_belief.mu)=%d.\n', size(prev_belief.mu));
assert(size(prev_belief.sig, 1) == size(prev_belief.sig, 2));
num_state_dims = length(prev_belief.mu);
assert(length(future_times) == prod(size(future_times)));
prediction_length = length(future_times);

% Unfiltered estimate of the current state
A = ekf_get_mat_A(dt);
Q = ekf_get_cov_Q(dt);
mu_hat = A * prev_belief.mu;
sig_hat = A * prev_belief.sig * A' + Q;

% Kalman gain
C = ekf_get_mat_C(prev_belief.mu);
R = ekf_get_cov_R(det);
% fprintf('Reciprocal condition number = %d\n', rcond(C * sig_hat * C' + R));
new_belief.innov_cov = C * sig_hat * C' + R;
K = sig_hat * C' * inv(new_belief.innov_cov);

% Filtered estimate of the current state
new_belief.innov = m - ekf_measurement_from_state(mu_hat);
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

end % function ekf
