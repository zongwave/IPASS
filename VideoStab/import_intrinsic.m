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

function [fc, cc, alpha_c, kc, fc_error, cc_error, alpha_c_error, kc_error, frame_size] = import_intrinsic(file)

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

    %-- Focal length:
    %fc = [ 1754.975680239204800 ; 1751.235395306768200 ];
    fc = getfield(calib_result, 'fc');
    
    %-- Principal point:
    %cc = [ 980.930704001504180 ; 533.190157376837190 ];
    cc = getfield(calib_result, 'cc');

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
