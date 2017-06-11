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
    elseif strcmp(field, 'rot_mat')
        projective2d = rot_mat.rot_mat();
    end

    trans_x = projective2d(1, 3, :);
    t_x{i} = trans_x(:);
    trans_y = projective2d(2, 3, :);
    t_y{i} = trans_y(:);

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

curve_len = 80;

figure();
for i=1: num_of_mat
    plot(t_y{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'T_Y'], [filename{2} 'T_Y']);

figure();
for i=1: num_of_mat
    plot(t_x{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'T_X'], [filename{2} 'T_X']);

figure();
for i=1: num_of_mat
    plot(s_y{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'S_Y'], [filename{2} 'S_Y']);

figure();
for i=1: num_of_mat
    plot(s_x{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'S_X'], [filename{2} 'S_X']);

figure();
for i=1: num_of_mat
    plot(r_y{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'R_Y'], [filename{2} 'R_Y']);

figure();
for i=1: num_of_mat
    plot(r_x{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'R_X'], [filename{2} 'R_X']);

figure();
for i=1: num_of_mat
    plot(p_y{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'P_Y'], [filename{2} 'P_Y']);

figure();
for i=1: num_of_mat
    plot(p_x{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'P_X'], [filename{2} 'P_X']);

figure();
for i=1: num_of_mat
    plot(h_w{i}(1:curve_len), 'LineWidth', 2);
    hold on
end
legend([filename{1} 'H_W'], [filename{2} 'H_W']);

end
