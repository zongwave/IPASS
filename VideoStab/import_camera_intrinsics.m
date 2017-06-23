%   import_camera_intrinsics.m - import camera calibration data to get
%   intrinsic parameters
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

function [fc, cc, alpha_c, kc, fc_error, cc_error, alpha_c_error, kc_error, frame_size] = import_camera_intrinsics(file)

clc;
clear;

if (nargin < 1)
    [filename, pathname] = uigetfile( ...
    {'*.mat','MAT files (*.mat)'; ...
       '*.*',  'All Files (*.*)'}, ...
       'Pick camera calibration file');

    calib_result = load(fullfile(pathname, filename));
    field = fieldnames(calib_result);

    % Intrinsic Camera Parameters

    % Color CameraIntrinsics by Tango API
    % image_width: 1920, image_height :1080, 
    % fx: 1750.517953, fy: 1752.121844, 
    % cx: 961.158431, cy: 543.400556, 
    % image_plane_distance: 1.823456.
    
    % Color Camera Frame with respect to IMU Frame, 
    % Position: 0.046403, -0.007203, -0.003940. 
    % Orientation: 0.007697, 0.999966, -0.002460, 0.001833


    %-- Focal length:
    %fc = [ 1754.975680239204800 ; 1751.235395306768200 ];
    fc = getfield(calib_result, 'fc');
%     fc = [1750.517953; 1752.121844];

    %-- Principal point:
    %cc = [ 980.930704001504180 ; 533.190157376837190 ];
    cc = getfield(calib_result, 'cc');
%     cc = [961.158431; 543.400556];

    %-- Skew coefficient:
    %alpha_c = 0.000000000000000;
    alpha_c = getfield(calib_result, 'alpha_c');

    %-- Distortion coefficients:
    %kc = [ 0.125138728392555 ; -0.370741771854929 ; -0.002331438922038 ; 0.005065576610383 ; 0.000000000000000 ];
    kc = getfield(calib_result, 'kc');

    %-- Focal length uncertainty:
    %fc_error = [ 44.984729995744409 ; 45.109724834116975 ];
    fc_error = getfield(calib_result, 'fc_error');

    %-- Principal point uncertainty:
    %cc_error = [ 13.861062313971287 ; 8.157315653129363 ];
    cc_error = getfield(calib_result, 'cc_error');

    %-- Skew coefficient uncertainty:
    %alpha_c_error = 0.000000000000000;
    alpha_c_error = getfield(calib_result, 'alpha_c_error');

    %-- Distortion coefficients uncertainty:
    %kc_error = [ 0.016094474616848 ; 0.082081733828857 ; 0.001497247209278 ; 0.002034878656146 ; 0.000000000000000 ];
    kc_error = getfield(calib_result, 'kc_error');

    %-- Image size:
    %nx = 1920;
    %ny = 1080;
    nx = getfield(calib_result, 'nx');
    ny = getfield(calib_result, 'ny');
    frame_size = [nx; ny];

end
