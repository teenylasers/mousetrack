%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function generate_track_suite
%
% Generate a suite of tracks as the basis for test suite.

function generate_test_suite()

% Generate a test suite for single-target tracking
num_stt = 10;
generate_stt_suite(num_stt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function generate_single_tracks_suite
%
% Generate a suite of single tracks to test/evaluate single-target tracking.
%
% Input:
% N = number of tests in this suite
%
% Output:
% tracks = the set of N test tracks, stored as an array of track structs. tracks is also
% saved to "track_suits_stt.mat".

function tracks = generate_stt_suite(num_tests)
global x_axis
global y_axis
global dt
tracks = [];
for i=1:num_tests
  % Estimate the amount of time a target takes to traverse the 2D space.
  %  - average target speed: 15 m/s ~ 33.5 mph
  %  - track length ~ 600 m
  %  - traversal time ~ 40 s
  %  - average num updates N = 400
  %  - add round(randn) to slightly randomize target speed
  N = 600/15/dt + round(randn(1) * 22);
  tracks = [tracks track_generator(x_axis, y_axis, N, dt)];
end
save('test_suite_stt_tracks', 'x_axis', 'y_axis', 'dt', 'tracks', 'num_tests');