%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function kf_convert_detection_to_measurement

% Convert radar detection det {r, theta, rdot} into the measurement vector
% m = [r*cos(theta); r*sin(theta); r*rdot]. Used in linear kf model.

function m = kf_convert_detection_to_measuremenrt(det)
m = [-det.r * sin(det.theta); det.r * cos(det.theta); det.r * det.rdot];