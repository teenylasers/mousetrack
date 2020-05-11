%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function mvnrnd
%
%  R = mvnrnd(mu, sig)
% dxN        dx1  dxd
% Return a matrix R of N random d-dimensional vectors from a multivariate Gaussian
% distribution with mean mu and covariance matrix sig.

function R = mvnrnd(mu, sig, N)

% Check input
assert(size(mu,2)==1, 'mu should be a column vector.');
assert(size(sig,1)==size(sig,2), 'sig should be a square matrix.');
d = size(mu,1);
assert(size(sig,1)==d,'Dimensions of mu and sig do not match.');

% Generate R
R = [];
[Q, Lambda] = eig(sig);
for ii = 1:N
  y = randn(d,1);
  x = Q*sqrt(Lambda)*y + mu;
  R = [R x];
end

end
