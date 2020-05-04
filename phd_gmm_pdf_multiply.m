%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function phd_gmm_pdf_multiply
%
% Implement the PDF multiplication in the numerator in the PHD-GMM filter.
%
% Input:
% ustate - {mu, sig, w}, a gaussian from the unfiltered estimation of the current state
% m - {x, y, r*rdot}, a measurement from the list of current measurements
% det - {r, theta, rdot}, the detection corresponding to m

function res = phd_gmm_pdf_multiply(ustate, m, det)

% Check input
assert(length(ustate.mu) == prod(size(ustate.mu)), ...
       'size(ustate.mu)=%d.\n', size(ustate.mu));
assert(size(ustate.sig, 1) == size(ustate.sig, 2));
ndims = length(ustate.mu); % state vector dimensions

%%%%%%  Implementation using extended Kalman filter  %%%%%%

% TODO: is it correct to use ustate.mu to get C, instead of prev_belief.mu?

C = ekf_get_mat_C(ustate.mu);
R = ekf_get_cov_R(det);
K = ustate.sig*C'*inv(R+C*ustate.sig*C');

res.w = ustate.w * ...
        normpdf(ekf_measurement_from_state(ustate.mu), C*ustate.sig*C'+R, m);
res.mu = ustate.mu + K*(m - ekf_measurement_from_state(ustate.mu));
res.sig = (eye(ndims) - K*C)*ustate.sig;

end
