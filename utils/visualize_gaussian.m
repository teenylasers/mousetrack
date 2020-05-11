%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function visualize_gaussian
%


function visualize_gaussian(mu, sig)

% Check that the input describes a 2D gaussian
assert(prod(size(mu))==length(mu), 'mu must be a vector.')
assert(length(mu)==2, 'mu needs to be a 2D vector.');
if (size(mu,1)==1)
  % mu is a row vector, convert to column vector for this function
  mu = mu';
end
assert(size(sig,1)==2 && size(sig,2)==2, 'sig must be a 2x2 matrix.');

% Draw confidence ellipse with particle swarm
ellipse = gaussian_confidence_ellipse(mu, sig);
dots = mvnrnd(mu, sig);
plot(ellipse(1,:), ellipse(2,:),'-');
hold;
plot(dots(1,:), dots(2,:), '.');

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function confidence_ellipse
%
% Given a 2D multivariate gaussian, defined by {mu, sig}, return a series of (x,y) pairs
% that draw out the confidence ellipse.
%
% Return val: ellipse, consists of 100 columns of [x;y] pairs.

function ellipse = gaussian_confidence_ellipse(mu, sig, s)

% Eigendecomposition of the covariance matrix sig to get the major and minor axis of the
% confidence ellipse.
[V, D] = eig(sig);
[major_index, tmp] = find(D == max(max(D)));
major_axis_vec = V(:, major_index);
major_axis_cov = D(major_index, major_index);
if (major_index==2); minor_index=1; else; minor_index=2; end
minor_axis_cov = D(minor_index, minor_index);

% Rotation matrix to tilt the ellipse due to correlation
angle = atan2(major_axis_vec(2), major_axis_vec(1));
R = [cos(angle) -sin(angle);
     sin(angle) cos(angle)];

% If s, the chi2 probability is not provided. Set it assuming 95% confidence interval.
if(nargin == 2); s = sqrt(5.991); end

% Return [x;y] pairs that form the confidence ellipse
theta = linspace(0,2*pi);
x = s*sqrt(major_axis_cov) * cos(theta);
y = s*sqrt(minor_axis_cov) * sin(theta);
ellipse = R * [x; y] + [mu(1); mu(2)];

end
