function [calib_param error] = camera_calibraion_matrix()

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
calib_param = [fc; cc; alpha_c; readout_time; gyro_delay; gyro_drifft];

delta = [1.1; 1.1];
min_gradient = [0.001; 0.001];

rot_mat = zeros(3, 3, size(gyro_quat, 1));
error = matching_error(frame_idx, p0', p1', frame_time, frame_size, gyro_pos', gyro_quat', gyro_time, calib_param, rot_mat);

while (any(abs(delta) > min_gradient) && error > 1.0)
    for d=1:numel(delta)
        calib_param(d) = calib_param(d) + delta(d);
        nex_error = matching_error(frame_idx, p0', p1', frame_time, frame_size, gyro_pos', gyro_quat', gyro_time, calib_param, rot_mat);
        if (nex_error > error)
            calib_param(d) = calib_param(d) - delta(d);
            delta(d) = -delta(d)/3;
        end
        error = nex_error;
        display(error);
    end
end

display(error);