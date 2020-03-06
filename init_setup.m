%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function init_setup
%
% Initial set-up of coordinate systems, global constants, and file names.

function init_setup()

% Set up the 2D space, i.e. the cartesian global frame.
global x_axis
global y_axis
x_axis.extent = 450;
y_axis.extent = x_axis.extent/2;
x_axis.min = -x_axis.extent/2;
x_axis.max = x_axis.min + x_axis.extent;
y_axis.min = 0;
y_axis.max = y_axis.min + y_axis.extent;

% Set up the radar frame with respect to the global frame.
% Transformation of a point between the radar frame b' and the global frame b is given by
%     b  =  p + Rb'
%     b' =  R^T(b - p)
global radar_coords
radar_coords.p = [(x_axis.max + x_axis.min)/2; y_axis.min];
radar_coords.bearing = pi/2;
radar_coords.R = ...
    [ cos(radar_coords.bearing) -sin(radar_coords.bearing) ; ...
      sin(radar_coords.bearing)  cos(radar_coords.bearing) ];

% Detection update frequency = 10 Hz
global dt
dt = 0.1;
