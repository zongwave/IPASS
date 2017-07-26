%   show_bayer_raw.m - convert bayer image to RGB
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

function show_bayer_raw(width, heigh, bit_depth, bayer_pattern)

clear;
clc;
close all;

[filename, pathname] = uigetfile( ...
{ '*.raw','RAW files (*.raw)'; ...
  '*.bmp','Bitmap files (*.bmp)'; ...
  '*.tif','TIF files (*.tif)'; ...
  '*.jpg','JPEG files (*.jpg)'; ...
   '*.*',  'All Files (*.*)'}, ...
   'Pick a file');

if (nargin < 3)
%     width = 2048;
%     heigh = 1092;
    width = 1920;
    heigh = 1080;
    bit_depth = 10;
    bayer_pattern = 'rggb';
end

fullfilename = fullfile(pathname, filename);
fip_raw = fopen(fullfilename,'rb');  

[RAW, num] = fread(fip_raw, inf, 'uint16');
pixel_count = width * heigh;
frame_count = num / pixel_count;

fclose(fip_raw);

bayer_vid_name = [fullfilename '_bayer'];
vidObj = VideoWriter([bayer_vid_name '.mp4'], 'MPEG-4');
open(vidObj);

for frame_idx=1: frame_count
    frame_idx
    for pix_idx=1 : pixel_count
        temp(pix_idx) = RAW(pix_idx + (frame_idx-1) * pixel_count);
        bayer(pix_idx) = double(temp(pix_idx) / (bitshift(1, bit_depth) - 1));
    end

    bayer = reshape(bayer, width, heigh);
    bayer = bayer';

    RGB = demosaic(uint8(255 * bayer), bayer_pattern);
    writeVideo(vidObj, RGB)
end
close(vidObj);

bayer_filename = [fullfilename '_frame.raw'];
fip_bayer = fopen(bayer_filename,'w');  
fwrite(fip_bayer, temp, 'uint16');
fclose(fip_bayer);

imwrite(RGB, [fullfilename '.bmp']);
figure();
imshow(RGB);

% for i=1 : 2: (pixel_count - width - 1)
%     tempR = RAW(i);
%     tempGr = RAW(i + 1);
%     tempGb = RAW(i + width);
%     tempB = RAW(i + width + 1);
% 
%     R(i) = double(tempR / (bitshift(1, bit_depth) - 1));
%     R(i+1) = R(i);
%     R(i + width) = R(i);
%     R(i + width + 1) = R(i);
%     
%     G(i) = double((tempGr + tempGb) / (2 * (bitshift(1, bit_depth) - 1)));
%     G(i + 1) = G(i);
%     G(i + width) = G(i);
%     G(i + width + 1) = G(i);
%     
%     B(i) = double(tempB / (bitshift(1, bit_depth) - 1));
%     B(i + 1) = B(i);
%     B(i + width) = B(i);
%     B(i + width + 1) = B(i);
% end
% 
% R = reshape(R, width, heigh);
% G = reshape(G, width, heigh);
% B = reshape(B, width, heigh);
% 
% R = R';
% G = G';
% B = B';
% 
% RGB = cat(3, R, G, B);

