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

function [vid_frame, frame_count, frame_rate, duration, vid_width, vid_height] = import_video(file)

clc;
clear;

if (nargin < 1)
    [filename, pathname] = uigetfile( ...
    {'*.mp4','MP4 files (*.mp4)'; ...
       '*.mpg','MPEG files (*.mpg)'; ...
       '*.mov','MOV files (*.mov)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick original shaky video file');

vidObj = VideoReader(fullfile(pathname, filename));

frame_count  = 0;
frame_rate = vidObj.FrameRate;
vid_height = vidObj.Height;
vid_width = vidObj.Width;
duration = vidObj.Duration;

figure();
title('Original Video');
currAxes = axes;
while hasFrame(vidObj)
    frame_count = frame_count + 1;
    vid_frame(frame_count).cdata = readFrame(vidObj);
    image(vid_frame(frame_count).cdata, 'Parent', currAxes);
    currAxes.Visible = 'off';
    pause(0.001/vidObj.FrameRate);
end

end
