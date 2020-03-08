%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function measurement_generator_kf_debug
%
% Generates measurements based on kalman filter model, that corresponds to the time index
% ti in the track
%

function m = measurement_generator_ekf_debug(track, ti)
s = [track.x(ti); track.vx(ti); track.y(ti); track.vy(ti)];
R = kf_get_cov_R();
m = ekf_measurement_from_state(s) + ...
    get_random_gaussian_distr_vectors(zeros(size(R,1),1),R,1);
end
