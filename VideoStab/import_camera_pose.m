%   import_camera_pose.m - import camera pose data acquired from Gyroscope
%   & Accelerometer
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

function [ts, translation, quatern] = import_camera_pose(file)

clear;
clc;
close all

if (nargin < 1)
    [filename, pathname] = uigetfile( ...
    {'*.data','DATA files (*.data)'; ...
       '*.csv','CSV files (*.csv)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick Gyro & Acc 6-DOF file');
end

[pathstr, name, ext] = fileparts(filename)

if  strcmp(ext, '.data')
        fid = fopen(fullfile(pathname, filename));
else
   fid = fopen('gyro.data');
end;

if  strcmp(ext, '.csv')
    gyro_data = csvread(fullfile(pathname, filename));

    figure();
    plot(gyro_data(:, 2), 'r', 'LineWidth', 2);
    hold on
    plot(gyro_data(:, 3), 'g', 'LineWidth', 2);
    hold on
    plot(gyro_data(:, 4), 'b', 'LineWidth', 2);
    hold on
    title('Gyro Angular Speed');
    legend('X', 'Y', 'Z');
else
    frewind(fid);
    frame = textscan(fid, '%s %s %s %s %s', 'Delimiter',':');
    fclose(fid);

    timestamp = frame{3};
    position = frame{4};
    orientation = frame{5};

    ts_count = size(timestamp);
    for i=1:ts_count(1)
        ts(i) = textscan(timestamp{i}, '%f');
    end
    ts = cell2mat(ts)';

    d = diff(ts);
    frame_gap = (d ./ min(d)) - 1;

    display(['avg frame rate: ' num2str(1/mean(d))]);
    display(['max frame rate: ' num2str(1/min(d))]);
    display(['max frame gap: ' num2str(max(frame_gap))]);
    display(['num frames dropped: ' num2str(sum(round(frame_gap)))]);

    for i=1:ts_count(1)
        translation{i} = textscan(position{i}, '%f %f %f', 'Delimiter', ',');
    end
    
if 0
    trans_x(1) = 0;
    trans_y(1) = 0;
    trans_z(1) = 0;
    for i=2:ts_count(1)
        trans_x(i) = (translation{i}{1} - translation{i-1}{1});
        trans_y(i) = (translation{i}{2} - translation{i-1}{2});
        trans_z(i) = (translation{i}{3} - translation{i-1}{3});
    end
else
    for i=1:ts_count(1)
        trans_x(i) = (translation{i}{1});
        trans_y(i) = (translation{i}{2});
        trans_z(i) = (translation{i}{3});
    end
end
    translation = [trans_x; trans_y; trans_z]';

    for i=1:ts_count(1)
        orientation{i} = textscan(orientation{i}, '%f %f %f %f', 'Delimiter', ',');
    end
    for i=1:ts_count(1)
        orient_x(i) = (orientation{i}{1});
        orient_y(i) = (orientation{i}{2});
        orient_z(i) = (orientation{i}{3});
        orient_w(i) = (orientation{i}{4});
    end
    quatern = [orient_x; orient_y; orient_z; orient_w]';

    figure();
    subplot(2, 3, 1);
    plot(trans_x, 'r', 'LineWidth', 2);
    hold on
    plot(trans_y, 'g', 'LineWidth', 2);
    hold on
    plot(trans_z, 'b', 'LineWidth', 2);
    hold on
    title('Gyro Translation');
    legend('Translation X', 'Translation Y', 'Translation Z');

    subplot(2, 3, 2);
    plot(orient_x, 'r', 'LineWidth', 2);
    hold on
    plot(orient_y, 'g', 'LineWidth', 2);
    hold on
    plot(orient_z, 'b', 'LineWidth', 2);
    hold on
    plot(orient_w, 'k', 'LineWidth', 2);
    hold on
    title('Gyro Orientation');
    legend('Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W');
end
eulerAngle = zeros(3, ts_count(1));
rotAxis = zeros(4, ts_count(1));
rotMatrix = zeros(3, 3, ts_count(1));


% %% Quaternion to Matrix
convertType = int32(0);

convert_rotation(convertType, quatern', eulerAngle, rotAxis, rotMatrix);
points = ones(ts_count(1), 3);
for i=1:size(rotMatrix, 3)
    points(i, :) = rotMatrix(:, :, i) * points(i, :)';
end

subplot(2, 3, 3);
plot3(points(:, 1), points(:, 2),  points(:, 3), 'r', 'LineWidth', 2);
hold on
title('Transform Points');
xlabel('X', 'Fontsize', 15);
ylabel('Y', 'Fontsize', 15);
zlabel('Z', 'Fontsize', 15);

% %% Quaternion to Euler angle
convertType = int32(1);

convert_rotation(convertType, quatern', eulerAngle, rotAxis, rotMatrix);

eulerAngle = (eulerAngle * 180 / 3.1415926);
    subplot(2, 3, 4);
plot(eulerAngle(1, :), 'r', 'LineWidth', 2);
hold on
plot(eulerAngle(2, :), 'g', 'LineWidth', 2);
hold on
plot(eulerAngle(3, :), 'b', 'LineWidth', 2);
hold on
title('Euler Angles');
legend('Euler Angle X(roll)', 'Euler Angle Y(pitch)', 'Euler Angle Z(yaw)');


% %% Quaternion to Rotation Axis
convertType = int32(2);

convert_rotation(convertType, quatern', eulerAngle, rotAxis, rotMatrix);

subplot(2, 3, 5);
plot(rotAxis(1, :), 'r', 'LineWidth', 2);
hold on
plot(rotAxis(2, :), 'g', 'LineWidth', 2);
hold on
plot(rotAxis(3, :), 'b', 'LineWidth', 2);
hold on
title('Rotation Axis');
legend('Rotation Axis X', 'Rotation Axis Y', 'Rotation Axis Z');

subplot(2, 3, 6);
plot(rotAxis(4, :), 'k', 'LineWidth', 2);
hold on
legend('Rotation Axis degree');




%%% Test by SpinCalc

%% Quaternion to Matrix
% rotMatrix_SC = SpinCalc('QtoDCM', quatern, 0.01, 1);
% points = ones(ts_count(1), 3);
% for i=1:size(rotMatrix_SC, 3)
%     points(i, :) = rotMatrix_SC(:, :, i) * points(i, :)';
% end
% 
% figure();
% subplot(2, 3, 3);
% plot3(points(:, 1), points(:, 2),  points(:, 3), 'r', 'LineWidth', 2);
% hold on
% title('SpinCalc Transform Points');
% xlabel('X', 'Fontsize', 15);
% ylabel('Y', 'Fontsize', 15);
% zlabel('Z', 'Fontsize', 15);

% %% Quaternion to Euler angles
% eulerAngle_SC = SpinCalc('QtoEA123', quatern, 0.01, 1);
% subplot(2, 3, 4);
% plot(eulerAngle_SC(:, 1), 'r', 'LineWidth', 2);
% hold on
% plot(eulerAngle_SC(:, 2), 'g', 'LineWidth', 2);
% hold on
% plot(eulerAngle_SC(:, 3), 'b', 'LineWidth', 2);
% hold on
% title('SpinCalc Euler Angles');
% legend('Euler Angles X', 'Euler Angles Y', 'Euler Angles Z');

% %% Quternion to rotation vector
% rotAxis_SC = SpinCalc('QtoEV', quatern, 0.01, 1);
% subplot(2, 3, 5);
% plot(rotAxis_SC(:, 1), 'r', 'LineWidth', 2);
% hold on
% plot(rotAxis_SC(:, 2), 'g', 'LineWidth', 2);
% hold on
% plot(rotAxis_SC(:, 3), 'b', 'LineWidth', 2);
% hold on
% title('SpinCalc Rotation Axis');
% legend('Rotation Axis X', 'Rotation Axis Y', 'Rotation Axis Z');
% subplot(2, 3, 6);
% plot(rotAxis_SC(:, 4), 'k', 'LineWidth', 2);
% hold on
% legend('Rotation Axis degree');
% 
% end
