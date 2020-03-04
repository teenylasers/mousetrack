%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function convert_detection_to_measurement

% Convert radar detection det {r, theta, r_dot} into the measurement vector
% m = [r*cos(theta); r*sin(theta); r*r_dot]

function m = convert_detection_to_measuremenrt(det)
m = [-det.r * sin(det.theta); det.r * cos(det.theta); det.r * det.r_dot];
