%   video_stabilizaton.m - main function to launch video stabilization
%
%    Copyright (c) 2017 Intel Corporation
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
 
    homo_mat = cell2mat(getfield(proj_mat, cell2mat(field)));
    homo_mat = reshape(homo_mat, 3, 3, size(homo_mat, 2)/3);

    if strcmp(field, 'homo_mat')
        video_stab_interface(0, 0, 0, 0, 0, 0, homo_mat);
        rot_mat = homo_mat;
        matrix_type = 'Optical Flow Motion Stab';
    else
        rot_mat = homo_mat;
        matrix_type = 'Optical Flow';
    end
else

    % Intrinsic Camera Parameters
    [fc, cc, alpha_c, kc, fc_error, cc_error, alpha_c_error, kc_error, frame_size] = import_camera_intrinsics();

    % Extrinsic Camera Parameters
    [gyro_ts, gyro_pos, gyro_quat] = import_camera_pose();
    frame_ts = gyro_ts;
    readout_time = 0;
    gyro_delay = 0;
    gyro_drifft = 0;
    rot_mat = zeros(3, 3, size(gyro_ts, 1));

    calib_param = [fc; cc; alpha_c; readout_time; gyro_delay; gyro_drifft];
    video_stab_interface(frame_ts, frame_size, gyro_pos', gyro_quat', gyro_ts, calib_param, rot_mat);
    matrix_type = 'Gyro 3DOF';
end

[vid_name, vid_frame, frame_count, frame_rate, duration, vid_width, vid_height] = import_video();

if filename ~= 0
    save([vid_name '_' matrix_type '_rot.mat'], 'rot_mat');
else
    gyro_mat = rot_mat(:, :, 1:frame_count);
    save([vid_name '_' matrix_type '_rot.mat'], 'gyro_mat');
end

mat_size = size(rot_mat);
if max(mat_size) < frame_count
    warp_count = max(mat_size);
else
    warp_count = frame_count;
end

vidObj = VideoWriter([vid_name '_stabilized_' matrix_type '.mp4'], 'MPEG-4');
open(vidObj);

crop_ratio = 0.05;
cropped_x = vid_width*crop_ratio;
cropped_y = vid_height*crop_ratio;
cropped_width = vid_width*(1-2*crop_ratio);
cropped_height = vid_height*(1-2*crop_ratio);

figure();
title(['Stabilized Video:  '  matrix_type]);
currAxes = axes;
for i=1: frame_count
    if i <= warp_count
        warp_idx = i;
    else
        warp_idx = warp_count;
    end
    warp_mat = projective2d(inv(rot_mat(:, :, warp_idx)'));
%     rot_mat(:, :, warp_idx)

    outputView = imref2d([vid_height, vid_width]);
    stabilized = imwarp(vid_frame(i).cdata, warp_mat, 'OutputView', outputView);

    cropped = imcrop(stabilized, [cropped_x, cropped_y, cropped_width, cropped_height]);
    scaled = imresize(cropped, [vid_height, vid_width]);
    image(scaled, 'Parent', currAxes);
    currAxes.Visible = 'off';
    pause(1/frame_rate);
    writeVideo(vidObj, scaled)
end

close(vidObj);

end
