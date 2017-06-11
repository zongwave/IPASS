% Digital Video Stabilization using Gyroscope & Accelerometer (6-DOF)
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
% Copyright (C) 2017 Zong Wei <zongwave@hotmail.com>
%

function [optimal_calib] = sensor_calibraion()

clc;
clear;
close all;

[frame_count, frame_idx, p0, p1] = import_keypoints();
[gyro_time, gyro_pos, gyro_quat] = import_gyro_data();

frame_time = gyro_time;
% Intrinsic Camera Parameters
[fc, cc, alpha_c, kc, fc_error, cc_error, alpha_c_error, kc_error, frame_size] = import_intrinsic();

% focal_x; focal_y; orig_x; orig_y; readout_time; gyro_delay; gyro_drifft;
readout_time = 0;
gyro_delay = 0;
gyro_drifft = 0;
init_calib = [fc; cc; alpha_c; readout_time; gyro_delay; gyro_drifft];

rot_mat = zeros(3, 3, size(gyro_quat, 1));

%  optimal calibration parameters.

%  Set options for fminunc
options = optimset('GradObj', 'on', 'MaxIter', 400);

%  Run fminunc to obtain the optimal calibration parameter
% [optimal_calib, error] = ...
optimal_calib = fminunc(@(calib_param)(cost_function(calib_param, ...
                                   frame_idx, p0', p1', frame_time, frame_size, ...
                                   gyro_pos', gyro_quat', gyro_time, ...
                                   rot_mat)), ...
                                   init_calib);

display(optimal_calib);
display(init_calib);

end