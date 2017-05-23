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

function [frame_count, frame_idx, p0, p1] = import_keypoints(file)

if (nargin < 1)
    [filename, pathname] = uigetfile( ...
    {'*.log','LOG files (*.log)'; ...
       '*.data','DATA files (*.bmp)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick a file');

   fid = fopen(fullfile(pathname, filename));
else
   fid = fopen('calib.log');
end;

frewind(fid);
calib = textscan(fid, '%s %d %s %f %f %s', 'Delimiter', ',');
fclose(fid);

frame_idx = calib{1};
point_idx = calib{2};
x0 = calib{3};
y0 = calib{4};
x1 = calib{5};
y1 = calib{6};

point_count = size(point_idx);
for i=1:point_count(1)
    fidx(i) = textscan(frame_idx{i}, '%*s %d');
    px0(i) = textscan(x0{i}, '%*s %*s %*s %f');
    py1(i) = textscan(y1{i}, '%f %*s %*s %*s %*s %*s');
end
frame_idx = cell2mat(fidx)';
frame_count = max(frame_idx);

x0 = cell2mat(px0)';
y1 = cell2mat(py1)';

p0 = double([ x0 y0 ]);
p1 = double([ x1 y1 ]);

end
