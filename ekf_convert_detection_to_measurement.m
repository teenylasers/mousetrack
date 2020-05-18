%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ekf_convert_detection_to_measurement

% Convert a cell array of radar detections det {r, theta, rdot} into a cell array of
% measurement vectors m = [r; theta; rdot]. Used in extended kf model.

function meas = ekf_convert_detection_to_measurement(det)
meas = {};
for i=1:length(det)
  d = det{i};
  m = [d.r; d.theta; d.rdot];
  meas{end+1} = m;
end
end % function ekf_convert_detection_to_measurement
