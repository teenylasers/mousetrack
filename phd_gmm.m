%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function phd_gmm
%
% Implement PHD filter using Gaussian mixture model
%
% Input:
% m = current measurements, a list of {x, y, r*rdot}
% det = current detections, a list of {r, theta, rdot}, used for cov R calculation
% dt = time since the last state estimate
% prev_belief = the previous belief function, a list of {mu, sig, innov, innov_cov, w}
% future_times = a list of times from now to predict the state
%
% Output:
% new_belief = the latest belief function, a list of {mu, sig, innov, innov_cov, w}
% predictions = a list future belief functions at future_times

function [new_belief, predictions] = phd_gmm(m, det, dt, prev_belief, future_times)

%%%%%%  Check input  %%%%%%
assert(length(prev_belief.mu) == prod(size(prev_belief.mu)), ...
    'size(prev_belief.mu)=%d.\n', size(prev_belief.mu));
assert(size(prev_belief.sig, 1) == size(prev_belief.sig, 2));
num_state_dims = length(prev_belief.mu);
assert(length(future_times) == prod(size(future_times)));
prediction_length = length(future_times);

%%%%%%  Model simplifications  %%%%%%
b = 0.2; % target_birth_probability
P_d = 1.0; % detection_probability <= 1.0
P_s = 1.0; % target_survival_probability <= 1.0
kappa = 1.0; % clutter model
mu_FA = 0; % false alarm rate

%%%%%%  Unfiltered estimate of the current state  %%%%%%
A = ekf_get_mat_A(dt);
Q = ekf_get_cov_Q(dt);

unfiltered = []; % unfiltered estimation
for i = 1:length(prev_belief)
  u.mu = A * prev_belief[i].mu;
  u.sig = A * prev_belief[i].sig * A' + Q;
  u.w = prev_belief[i].w;
  unfiltered = [unfiltered u];
  end
end

%%%%%%  Filtered new belief  %%%%%%

% denominator
denom = [];
for j = 1:length(m)
  for k = 1:length(unfiltered)
    C = efk_get_mat_C(unfiltered[k].mu);
    R = ekf_get_cov_R(det[j]);
    d.mu = ekf_measurement_from_state(unfiltered[k].mu);
    d.sig = C * unfiltered[k].sig * C' + R;
    d.w = unfiltered[k].w;
    denom = [denom d];
  end
end

% numerator
numer = [];
for j = 1:length(m)
  for k = 1:length(unfiltered)
    % The unfiltered state estimation and the measurement are in different
    % spaces, the multiplication of the 2 probability densities is not a
    % direct multiplication. phd_gmm_pdf_multiply() implements this
    % multiplication using specific methods.
    nu = phd_gmm_pdf_multiply(unfiltered[k], m[j], det[j]);
    numer = [numer nu];
  end
end

% TODO: incorporate P_d, kappa and mu_FA

new_belief = [];
predictions = [];

%%%%%                              TO DO NEXT                           %%%%%
%
% First, visualize numerator overlaid with denominator gaussian mixtures, before
% deciding how to compute and prune further