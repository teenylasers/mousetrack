%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function get_random_gaussian_distr_vectors
%
% Return N vectors v that follows a multivariate gaussian statistic with mean mu and
% covariance sig.

function v = get_random_gaussian_distr_vectors(mu, sig, N)
dim = length(mu);
L = chol(sig, 'lower');
v = [];
for i = 1:N
  y = randn(dim, 1);
  v = [v L*y+mu];
end
