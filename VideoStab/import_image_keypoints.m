%   import_image_keypoints.m - import image keypoints to do calibration
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

function [frame_count, frame_idx, p0, p1] = import_image_keypoints(file)

if (nargin < 1)
    [filename, pathname] = uigetfile( ...
    {'*.log','LOG files (*.log)'; ...
       '*.data','DATA files (*.bmp)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick keypoints file');

   fid = fopen(fullfile(pathname, filename));
else
   fid = fopen('keypoints.log');
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
