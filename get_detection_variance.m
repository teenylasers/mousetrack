%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function get_detection_variance

function var = get_detection_variance(det)

% TODO: set realistic magnitudes
var.sig_r = 2;
var.sig_theta = 3/180*pi;
var.sig_rdot = 0.5;

end
