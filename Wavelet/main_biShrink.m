function main_biShrink()

close all;
clc;
clear;

[filename, pathname] = uigetfile( ...
{  '*.bmp','BMP files (*.bmp)'; ...
   '*.png','PNG files (*.png)'; ...
   '*.jpg','JPG files (*.jpg)'; ...
   '*.*',  'All Files (*.*)'}, ...
   'Pick cleaned image file');

origfilename = fullfile(pathname, filename);
orig = imread(origfilename);
orig = double(orig);

[filename, pathname] = uigetfile( ...
{  '*.bmp','BMP files (*.bmp)'; ...
   '*.png','PNG files (*.png)'; ...
   '*.jpg','JPG files (*.jpg)'; ...
   '*.*',  'All Files (*.*)'}, ...
   'Pick noisy image file');

noisyfilename = fullfile(pathname, filename);
noisy = imread(noisyfilename);
noisy = double(noisy);

[pathstr, name, ext] = fileparts(noisyfilename);
fname = [pathstr '/' name '_BiShrink.bmp'];

[row, column, channel] = size(orig);

if channel == 3
    [PSNR_r, est_r] = main_dwt(orig(:, :, 1), noisy(:, :, 1));
    [PSNR_g, est_g] = main_dwt(orig(:, :, 2), noisy(:, :, 2));
    [PSNR_b, est_b] = main_dwt(orig(:, :, 3), noisy(:, :, 3));

    figure();
    subplot(1, 2, 1);
    imshow(noisy / 255.0);
    subplot(1, 2, 2);
    imshow(cat(3, est_r / 255.0, est_g / 255.0, est_b / 255.0));
    imwrite(cat(3, est_r / 255.0, est_g / 255.0, est_b / 255.0), fname);
else
    [PSNR_y, est_y] = main_dwt(orig, noisy);
    figure();
    subplot(1, 2, 1);
    imshow(noisy);
    subplot(1, 2, 2);
    imshow(est_y);
    imwrite(est_y, fname);
end
