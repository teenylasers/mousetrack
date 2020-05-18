%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function kf_convert_detection_to_measurement

% Convert a cell array of radar detections det {r, theta, rdot} into a cell array of
% measurement vectors m = [r*cos(theta); r*sin(theta); r*rdot]. Used in linear kf model.

function meas = kf_convert_detection_to_measurement(det)
meas = {};
for i=1:length(det)
  d = det{i};
  m = [-d.r * sin(d.theta); d.r * cos(d.theta); d.r * d.rdot];
  meas{end+1} = m;
end
end
