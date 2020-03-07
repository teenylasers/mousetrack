%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function measurement_generator_kalman_filter_debug
%
% Generates measurements based on kalman filter model, that corresponds to the time index
% ti in the track
%

function m = measurement_generator_kalman_filter_debug(track, ti)
s = [track.x(ti); track.vx(ti); track.y(ti); track.vy(ti)];
C = kalman_filter_get_matrix_C(s);
R = kalman_filter_get_covariance_R();
m = C*s + get_random_gaussian_distr_vectors(zeros(size(C,1),1),R,1);