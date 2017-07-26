%   analyze_projective2d.m - Plot projection matrix to analyze
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

function analyze_projective2d(file)

clc;
clear;
close all;

if (nargin < 1)
    [filename, pathname, filterindex] = uigetfile( ...
    {'*.mat','MAT files (*.mat)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick projective 2D file', ...
       'MultiSelect', 'on');

if (iscell(filename) == 0)
    if filename == 0
        num_of_mat = 0;
    else
        num_of_mat = 1;
    end
else
    num_of_mat = length(filename);
end

for i=1: num_of_mat
    if (iscell(filename) == 0)
        rot_mat = load(fullfile(pathname, filename));
    else
        rot_mat = load(fullfile(pathname, filename{i}));
    end

    field = fieldnames(rot_mat);
    if strcmp(field, 'gyro_mat')
        projective2d = rot_mat.gyro_mat;
        curve{i} = 'Gyro 3DOF';
   elseif strcmp(field, 'rot_mat')
        projective2d = rot_mat.rot_mat();
        curve{i} = 'Optical Flow';
    end

    trans_x = projective2d(1, 3, :);
    t_x{i} = trans_x(:);
    trans_y = projective2d(2, 3, :);
    t_y{i} = trans_y(:);

    curve_len{i} = size(t_y{i}, 1);

    scale_x = projective2d(1, 1, :);
    s_x{i} = scale_x(:);
    scale_y = projective2d(2, 2, :);
    s_y{i} = scale_y(:);

    rot_x = projective2d(1, 2, :);
    r_x{i} = rot_x(:);
    rot_y = projective2d(2, 1, :);
    r_y{i} = rot_y(:);

    proj_x = projective2d(3, 1, :);
    p_x{i} = proj_x(:);
    proj_y = projective2d(3, 2, :);
    p_y{i} = proj_y(:);

    homo_w = projective2d(3, 3, :);
    h_w{i} = homo_w(:);
end

if curve_len{1} > 270
    curve_step = 3;
elseif curve_len{1} > 180
    curve_step = 2;
else
    curve_step = 1;
end

figure();
subplot(3, 3, 1);
for i=1: num_of_mat
    plot(s_x{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  S_X'], [curve{2} '  S_X']);

subplot(3, 3, 2);
for i=1: num_of_mat
    plot(r_x{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  R_X'], [curve{2} '  R_X']);

subplot(3, 3, 3);
for i=1: num_of_mat
    plot(t_x{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  T_X'], [curve{2} '  T_X']);

subplot(3, 3, 4);
for i=1: num_of_mat
    plot(r_y{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  R_Y'], [curve{2} '  R_Y']);

subplot(3, 3, 5);
for i=1: num_of_mat
    plot(s_y{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  S_Y'], [curve{2} '  S_Y']);

subplot(3, 3, 6);
for i=1: num_of_mat
    plot(t_y{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  T_Y'], [curve{2} '  T_Y']);

subplot(3, 3, 7);
for i=1: num_of_mat
    plot(p_x{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  P_X'], [curve{2} '  P_X']);

subplot(3, 3, 8);
for i=1: num_of_mat
    plot(p_y{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  P_Y'], [curve{2} '  P_Y']);

subplot(3, 3, 9);
for i=1: num_of_mat
    plot(h_w{i}(1: curve_step: curve_len{i}), 'LineWidth', 2);
    hold on
end
legend([curve{1} '  H_W'], [curve{2} '  H_W']);

end
