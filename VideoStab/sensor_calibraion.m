%   sensor_calibration.m - Optimal camera & gyrocope calibration parameters
%   by build in function 'fminunc'
%
%   Licensed under the Apache License, Version 2.0 (the "License");
%   you may not use this file except in compliance with the License.
%   You may obtain a copy of the License at
%
%        http://www.apache.org/licenses/LICENSE-2.0
%
%   Unless required by applicable law or agreed to in writing, software
%   distributed under the License is distributed on an "AS IS" BASIS,
%   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%   See the License for the specific language governing permissions and
%   limitations under the License.
%
%   Author: Zong Wei <wei.zong@intel.com>
%

function [optimal_calib] = sensor_calibraion()

clc;
clear;
close all;

[frame_count, frame_idx, p0, p1] = import_image_keypoints();
[gyro_time, gyro_pos, gyro_quat] = import_camera_pose();

frame_time = gyro_time;
% Intrinsic Camera Parameters
[fc, cc, alpha_c, kc, fc_error, cc_error, alpha_c_error, kc_error, frame_size] = import_camera_intrinsics();

% focal_x; focal_y; orig_x; orig_y; readout_time; gyro_delay; gyro_drifft;
readout_time = 0;
gyro_delay = 0;
gyro_drifft = 0;
init_calib = [fc; cc; alpha_c; readout_time; gyro_delay; gyro_drifft];

rot_mat = zeros(3, 3, size(gyro_quat, 1));

%  optimal calibration parameters.

%  Set options for fminunc
% options = optimset('GradObj', 'on', 'MaxIter', 400);
options = optimoptions(@fminunc, ...
                       'Display','iter', ...
                       'Algorithm','quasi-newton', ...
                       'MaxFunctionEvaluations',8000);

%  Run fminunc to obtain the optimal calibration parameter
optimal_calib = fminunc(@(calib_param)(cost_function(calib_param, ...
                                   frame_idx, p0', p1', frame_time, frame_size, ...
                                   gyro_pos', gyro_quat', gyro_time, ...
                                   rot_mat)), ...
                                   init_calib, options);

display(optimal_calib);
display(init_calib);

end
