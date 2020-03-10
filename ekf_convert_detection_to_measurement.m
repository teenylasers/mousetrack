%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_convert_detection_to_measurement

% Convert radar detection det {r, theta, rdot} into the measurement vector
% m = [r*cos(theta); r*sin(theta); r*rdot]. Used in linear kf model.

function m = ekf_convert_detection_to_measurement(det)
m = [det.r; det.theta; det.rdot];
end % function ekf_convert_detection_to_measurement
