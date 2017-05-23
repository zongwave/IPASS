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

function video_stabilization(file)

clc
clear
close all

if (nargin < 1)
    [filename, pathname] = uigetfile( ...
    {'*.mat','Mat files (*.mat)'; ...
       '*.data','DATA files (*.data)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick pre-calculated rotation Matrix file');

if filename ~= 0
    proj_mat = load(fullfile(pathname, filename));
    field = fieldnames(proj_mat);
 
    rot_mat = cell2mat(getfield(proj_mat, cell2mat(field)));
    rot_mat = reshape(rot_mat, 3, 3, size(rot_mat, 2)/3);
    matrix_type = 'Optical Flow ...';
else

    % Intrinsic Camera Parameters
    [fc, cc, alpha_c, kc, fc_error, cc_error, alpha_c_error, kc_error, frame_size] = import_intrinsic();

    % Extrinsic Camera Parameters
    [gyro_time, gyro_pos, gyro_quat] = import_gyro_data();
    frame_time = gyro_time;
    readout_time = 0;
    gyro_delay = 0;
    gyro_drifft = 0;
    rot_mat = zeros(3, 3, size(gyro_time, 1));
    
    calib_param = [fc; cc; alpha_c; readout_time; gyro_delay; gyro_drifft];
    find_homography(frame_time, frame_size, gyro_pos', gyro_quat', gyro_time, calib_param, rot_mat);
    matrix_type = 'Gyro 3-DOF...';
end

[vid_frame, frame_count, frame_rate, duration, vid_width, vid_height] = import_video();

mat_size = size(rot_mat);
if max(mat_size) < frame_count
    warp_count = max(mat_size);
else
    warp_count = frame_count;
end

figure();
title(['Stabilized Video:  '  mat2str(matrix_type)]);
currAxes = axes;
for i=1: frame_count
    if i <= warp_count
        warp_idx = i;
    else
        warp_idx = warp_count;
    end
    rot_mat(:, :, warp_idx)
    warp_mat = projective2d(rot_mat(:, :, warp_idx)');
    stabilized = imwarp(vid_frame(i).cdata, warp_mat);
    image(stabilized, 'Parent', currAxes);
    currAxes.Visible = 'off';
    pause(1/frame_rate);
end

end
