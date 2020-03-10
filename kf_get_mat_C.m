%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function kf_get_mat_C
%
% Measurement model
%     m = [x; y; x*xdot+y*ydot ~ r*rdot]
%     m = C * s + N(0,R)
% where N(0,R) is the gaussian measurement noise with zero-mean and covariance R.

function C = kf_get_mat_C(s)
C = [   1    0    0    0;
        0    0    1    0;
      s(2) s(1) s(4) s(3)];
