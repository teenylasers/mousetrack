%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function phd_gmm
%
% Implement PHD filter using Gaussian mixture model
%
% Input:
% m = current measurements, a cell array of {r, theta, rdot}, also used for cov R calculation
% dt = time since the last state estimate
% prev_belief = the previous belief function, a cell array of {mu, sig, innov, innov_cov, w}
% future_times = a list of times from now to predict the state
%
% Output:
% new_belief = the latest belief function, a list of {mu, sig, innov, innov_cov, w}
% predictions = a list future belief functions at future_times

function [new_belief, predictions] = phd_gmm(m, dt, prev_belief, future_times)

%%%%%%  Check input  %%%%%%
if length(prev_belief)>0
  for i = 1:length(prev_belief)
    assert(length(prev_belief{i}.mu) == prod(size(prev_belief{i}.mu)), ...
           'size(prev_belief.mu)=(%d, %d).\n', ...
           size(prev_belief{i}.mu, 1), size(prev_belief{i}.mu, 2));
    assert(size(prev_belief{i}.sig, 1) == size(prev_belief{i}.sig, 2));
  end
end
assert(length(future_times) == prod(size(future_times)));
prediction_length = length(future_times);

%%%%%%  Model assumptions & simplifications  %%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%     TO DO    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% target_birth_probability
% TODO: How to handle target birth
% TODO: setting of sig and w depends on what causes target birth.
const_b_sig = 10;
const_b_w = 0.02;


P_d = 1.0; % detection_probability <= 1.0
P_s = 1.0; % target_survival_probability <= 1.0
kappa = 1.0; % clutter model
mu_FA = 0; % false alarm rate

%%%%%%  Unfiltered estimate of the current state  %%%%%%
A = ekf_get_mat_A(dt);
Q = ekf_get_cov_Q(dt);

unfiltered = {}; % unfiltered estimation
for i = 1:length(prev_belief)
  u.mu = A * prev_belief{i}.mu;
  u.sig = A * prev_belief{i}.sig * A' + Q;
  u.w = prev_belief{i}.w;
  unfiltered{end+1} = u;
end

%%%%%%  Filtered new belief  %%%%%%

% denominator
denom = 0;
for j = 1:length(m)
  % From spontaneous target birth
  b.mu = ekf_measurement_to_initial_state(m{j});
  b.sig = eye(length(b.mu))*const_b_sig;
  b.w = const_b_w;
  C = ekf_get_mat_C(b.mu);
  R = ekf_get_cov_R(m{j});
  d.mu = ekf_measurement_from_state(b.mu);
  d.sig = C * b.sig * C' + R;
  d.w = b.w;
  val = d.w * normpdf(d.mu, d.sig, m{j});
  assert(~isnan(val), 'new belief normalization error: val = nan');
  denom = denom + val;
  % From prev_beliefs
  for k = 1:length(unfiltered)
    C = ekf_get_mat_C(unfiltered{k}.mu);
    R = ekf_get_cov_R(m{j});
    d.mu = ekf_measurement_from_state(unfiltered{k}.mu);
    d.sig = C * unfiltered{k}.sig * C' + R;
    d.w = unfiltered{k}.w;
    val = d.w * normpdf(d.mu, d.sig, m{j});
    assert(~isnan(val), 'new belief normalization error: val = nan');
    denom = denom + val;
  end
end
assert(~isnan(denom), 'new belief normalization error: denom = nan');
assert(denom~=0, 'new belief normalization error: denom = 0');

% numerator
% numerator.w / denom -> new_belief
new_belief = {};
for j = 1:length(m)
  % From spontaneous target birth
  b.mu = ekf_measurement_to_initial_state(m{j});
  b.sig = eye(length(b.mu))*const_b_sig;
  b.w = const_b_w;
  numer = phd_gmm_pdf_multiply(b, m{j});
  numer.w = numer.w / denom;
  new_belief{end+1} = numer;
  % From prev_beliefs
  for k = 1:length(unfiltered)
    % The unfiltered state estimation and the measurement are in different
    % spaces, the multiplication of the 2 probability densities is not a
    % direct multiplication. phd_gmm_pdf_multiply() implements this
    % multiplication using specific methods.
    numer = phd_gmm_pdf_multiply(unfiltered{k}, m{j});
    numer.w = numer.w / denom;
    new_belief{end+1} = numer;
  end
end

% TODO: incorporate P_d, kappa and mu_FA
predictions = {};

%%%%%                              TO DO NEXT                           %%%%%
%
% First, visualize numerator overlaid with denominator gaussian mixtures, before
% deciding how to compute and prune further


end