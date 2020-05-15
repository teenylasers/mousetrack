%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Evaluate a multivariate normal (Gaussian) pdf with mean mu and covariance sig at x
% function y = normpdf(mu, sig, x)

function y = normpdf(mu, sig, x)

% Sanity check input
assert((size(x,1)==size(mu,1)) && (size(x,2)==size(mu,2)), ...
       'x and mu must have the same dimensions.');
assert(size(sig,1)==size(sig,2), 'sig must be a square matrix.');
if (size(mu,1)==1)
  col_vector = 0; % x and mu are row vectors
  assert(size(mu,2)==size(sig,1), 'mu and sig do not have matching dimensions.');
  x = x'; % Convert x and mu to column vectors
  mu = mu';
else
  assert(size(mu,2)==1, 'mu and x must be vectors.');
  assert(size(mu,1)==size(sig,1), 'mu and sig do not have matching dimensions.');
end

% Evaluate the gaussian PDF at x
k = size(sig, 1); % dimension of the multivariate gaussian
y = 1/sqrt(2^k*pi^k*det(sig)) * exp(-(1/2) * (x-mu)' * inv(sig) * (x-mu));

end
