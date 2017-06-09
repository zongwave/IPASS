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

function [ts, translation, quatern] = import_gyro_data(file)

clear;
clc;
close all

if (nargin < 1)
    [filename, pathname] = uigetfile( ...
    {'*.data','DATA files (*.data)'; ...
       '*.data','DATA files (*.bmp)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick Gyro 6-DOF file');

   fid = fopen(fullfile(pathname, filename));
else
   fid = fopen('gyro_data.txt');
end;

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
for i=1:ts_count(1)
    trans_x(i) = (translation{i}{1});
    trans_y(i) = (translation{i}{2});
    trans_z(i) = (translation{i}{3});
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
plot(trans_x, 'r', 'LineWidth', 2);
hold on
plot(trans_y, 'g', 'LineWidth', 2);
hold on
plot(trans_z, 'b', 'LineWidth', 2);
hold on
title('Gyro Translation');
legend('Translation X', 'Translation Y', 'Translation Z');

figure();
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

% eulerAngle = zeros(3, ts_count(1));
% rotAxis = zeros(4, ts_count(1));
% rotMatrix = zeros(3, 3, ts_count(1));
%
%
% %% Quaternion to Matrix
% convertType = int32(0);
%
% convert_rotation(convertType, quatern', eulerAngle, rotAxis, rotMatrix);
% points = ones(ts_count(1), 3);
% for i=1:size(rotMatrix, 3)
%     points(i, :) = rotMatrix(:, :, i) * points(i, :)';
% end
%
% figure();
% plot3(points(:, 1), points(:, 2),  points(:, 3), 'r', 'LineWidth', 2);
% hold on
% title('Transform Points');
% xlabel('X', 'Fontsize', 15);
% ylabel('Y', 'Fontsize', 15);
% zlabel('Z', 'Fontsize', 15);
%
% %% Quaternion to Euler angle
% convertType = int32(1);
%
% convert_rotation(convertType, quatern', eulerAngle, rotAxis, rotMatrix);
% eulerAngle = (eulerAngle * 180 / 3.1415926);
% figure();
% plot(eulerAngle(1, :), 'r', 'LineWidth', 2);
% hold on
% plot(eulerAngle(2, :), 'g', 'LineWidth', 2);
% hold on
% plot(eulerAngle(3, :), 'b', 'LineWidth', 2);
% hold on
% title('Euler Angles');
% legend('Euler Angle X(roll)', 'Euler Angle Y(pitch)', 'Euler Angle Z(yaw)');
%
%
% %% Quaternion to Rotation Axis
% convertType = int32(2);
%
% convert_rotation(convertType, quatern', eulerAngle, rotAxis, rotMatrix);
% figure();
% subplot(1,  2, 1);
% plot(rotAxis(1, :), 'r', 'LineWidth', 2);
% hold on
% plot(rotAxis(2, :), 'g', 'LineWidth', 2);
% hold on
% plot(rotAxis(3, :), 'b', 'LineWidth', 2);
% hold on
% title('Rotation Axis');
% legend('Rotation Axis X', 'Rotation Axis Y', 'Rotation Axis Z');
% subplot(1,  2, 2);
% plot(rotAxis(4, :), 'k', 'LineWidth', 2);
% hold on
% legend('Rotation Axis degree');
%
%


%% Quaternion to Matrix
% rotMatrix_SC = SpinCalc('QtoDCM', quatern, 0.01, 1);
% points = ones(ts_count(1), 3);
% for i=1:size(rotMatrix_SC, 3)
%     points(i, :) = rotMatrix_SC(:, :, i) * points(i, :)';
% end
%
% figure();
% plot3(points(:, 1), points(:, 2),  points(:, 3), 'r', 'LineWidth', 2);
% hold on
% title('SpinCalc Transform Points');
% xlabel('X', 'Fontsize', 15);
% ylabel('Y', 'Fontsize', 15);
% zlabel('Z', 'Fontsize', 15);
%
% %% Quaternion to Euler angles
% eulerAngle_SC = SpinCalc('QtoEA123', quatern, 0.01, 1);
% figure();
% plot(eulerAngle_SC(:, 1), 'r', 'LineWidth', 2);
% hold on
% plot(eulerAngle_SC(:, 2), 'g', 'LineWidth', 2);
% hold on
% plot(eulerAngle_SC(:, 3), 'b', 'LineWidth', 2);
% hold on
% title('SpinCalc Euler Angles');
% legend('Euler Angles X', 'Euler Angles Y', 'Euler Angles Z');
%
% %% Quternion to rotation vector
% rotAxis_SC = SpinCalc('QtoEV', quatern, 0.01, 1);
% figure();
% subplot(1,  2, 1);
% plot(rotAxis_SC(:, 1), 'r', 'LineWidth', 2);
% hold on
% plot(rotAxis_SC(:, 2), 'g', 'LineWidth', 2);
% hold on
% plot(rotAxis_SC(:, 3), 'b', 'LineWidth', 2);
% hold on
% title('SpinCalc Rotation Axis');
% legend('Rotation Axis X', 'Rotation Axis Y', 'Rotation Axis Z');
% subplot(1,  2, 2);
% plot(rotAxis_SC(:, 4), 'k', 'LineWidth', 2);
% hold on
% legend('Rotation Axis degree');

end
