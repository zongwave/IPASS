%   wavelet_denoise.m - image denoise using wavelet
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

function wavelet_denoise(wname, tname, tuning)


[vid_name, vid_frame, frame_count, frame_rate, duration, vid_width, vid_height] = import_video();


% THR = wthrmngr('dw2ddenoLVL','penalhi',C,S,ALFA) 
%       ALFA must be such that 2.5 < ALFA < 10
% THR = wthrmngr('dw2ddenoLVL','penalme',C,S,ALFA) 
%       ALFA must be such that 1.5 < ALFA < 2.5
% THR = wthrmngr('dw2ddenoLVL','penallo',C,S,ALFA) 
%       ALFA must be such that 1 < ALFA < 2
% THR = wthrmngr('dw2ddenoLVL','sqtwolog',C,S,SCAL)
% THR = wthrmngr('dw2ddenoLVL','sqrtbal_sn',C,S)

alpha = tuning;
glb = 0;

vidObj = VideoWriter([vid_name '_wavelet_denoise_' wname '_tuning_' tname '_' mat2str(tuning) '_global_thresh_' mat2str(glb)  '.mp4'], 'MPEG-4');
open(vidObj);

figure();
title([wname ' Wavelet denoised Video ' tname ' tuning ' mat2str(tuning)]);
currAxes = axes;

test_frame_count = 20;
for iFrame=1: frame_count
    input = vid_frame(iFrame).cdata;

    in_name = [vid_name '_' mat2str(iFrame)  '.bmp'];
    if iFrame == 1
        imwrite(input, in_name);
    end

    [row, col, channel] = size(input);

    if channel == 3
        input_yuv = rgb2yuv(input(:, :, 1), input(:, :, 2), input(:, :, 3));
    else
        input_yuv = rgb2yuv(input(:, :, 1), input(:, :, 1), input(:, :, 1));    
    end

    input_yuv = im2double(input_yuv);

    % Run local adaptive image denoising algorithm
    for iChannel = 1: channel
        output_yuv(:, :, iChannel) = 255 * denoising_dwt(input_yuv(:, :, iChannel), wname, tname, tuning);
    end

    if channel == 3
        output = yuv2rgb(output_yuv(:, :, 1), output_yuv(:, :, 2), output_yuv(:, :, 3), 'YUV444_8');
    else
        output = output_yuv(:, :, 1);   
    end
    output = im2double(output);
    
    out_name = [vid_name '_' mat2str(iFrame) '_wavelet_denoise_' wname '_threh_' tname '_' mat2str(tuning) '.bmp'];
    if iFrame == 1
        imwrite(output, out_name);
    end

    image(output, 'Parent', currAxes);
    currAxes.Visible = 'off';
    pause(1/frame_rate);
    writeVideo(vidObj, output)

    iFrame
end

close(vidObj);

end