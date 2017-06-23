%   import_video.m - import original video
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

function [fullname, vid_frame, frame_count, frame_rate, duration, vid_width, vid_height] = import_video(file)

clc;
clear;

if (nargin < 1)
    [filename, pathname] = uigetfile( ...
    {'*.mp4','MP4 files (*.mp4)'; ...
       '*.mpg','MPEG files (*.mpg)'; ...
       '*.mov','MOV files (*.mov)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick original shaky video file');

fullname = fullfile(pathname, filename);
vidObj = VideoReader(fullname);

frame_count  = 0;
frame_rate = vidObj.FrameRate;
vid_height = vidObj.Height;
vid_width = vidObj.Width;
duration = vidObj.Duration;

figure();
title('Original Video');
currAxes = axes;
while hasFrame(vidObj)
    vidFrame = readFrame(vidObj);
    frame_count = frame_count + 1;
    vid_frame(frame_count).cdata = vidFrame;
    image(vid_frame(frame_count).cdata, 'Parent', currAxes);
    currAxes.Visible = 'off';
    pause(0.001/vidObj.FrameRate);
end

end
