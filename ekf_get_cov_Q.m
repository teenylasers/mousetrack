%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_get_cov_Q
%
% State model
%     s = [x; xdot; y; ydot]
%     s1 = A * s + N(0,Q)
% where N(0,Q) is the gaussian process noise with zero-mean and covariance Q.

function Q = ekf_get_cov_Q(dt)
% Using Singer model for sampling time << acceleration/maneuvering time
% From Blackman Page 33 without derivation
sig_m = 50; % TODO: set realistic magnitudes
tau = 1;
Q = 2 * sig_m^2 / tau * ...
    [dt^5/20 dt^4/8 0 0;
     dt^4/8  dt^3/3 0 0;
     0 0 dt^5/20 dt^4/8;
     0 0 dt^4/8  dt^3/3];

% Q = [10 0 0 0;
%       0 1 0 0;
%       0 0 10 0;
%       0 0 0 1];

% Using constant velocity model,
% http://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf
% q = 1000;
% Q = q * [dt^3/3 dt^2/2   0      0;
%          dt^2/2   dt     0      0;
% 	   0       0  dt^3/3 dt^2/2;
% 	   0       0  dt^2/2   dt  ];

end % function ekf_get_cov_Q
