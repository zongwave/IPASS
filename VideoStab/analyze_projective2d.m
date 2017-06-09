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
        projective2d = rot_mat.rot_mat;
    end

    trans_x = projective2d(1, 3, :);
    trans_x = trans_x(:);
    trans_y = projective2d(2, 3, :);
    trans_y = trans_y(:);

    scale_x = projective2d(1, 1, :);
    scale_x = scale_x(:);
    scale_y = projective2d(2, 2, :);
    scale_y = scale_y(:);

    plot(trans_y);
    hold on
end

end
