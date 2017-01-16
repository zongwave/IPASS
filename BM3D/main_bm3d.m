function main_bm3d()

close all;

[filename, pathname] = uigetfile( ...
{  '*.bmp','BMP files (*.bmp)'; ...
   '*.png','PNG files (*.png)'; ...
   '*.jpg','JPG files (*.jpg)'; ...
   '*.*',  'All Files (*.*)'}, ...
   'Pick cleaned image file');

origfilename = fullfile(pathname, filename);
orig = im2double(imread(origfilename));

[filename, pathname] = uigetfile( ...
{  '*.bmp','BMP files (*.bmp)'; ...
   '*.png','PNG files (*.png)'; ...
   '*.jpg','JPG files (*.jpg)'; ...
   '*.*',  'All Files (*.*)'}, ...
   'Pick noisy image file');

noisyfilename = fullfile(pathname, filename);
noisy = im2double(imread(noisyfilename));

[pathstr, name, ext] = fileparts(noisyfilename);
fname = [pathstr '/' name '_BM3D.bmp'];

[row, column, channel] = size(orig);

if channel == 3
    [PSNR_r, est_r] = BM3D(orig(:, :, 1), noisy(:, :, 1));
    [PSNR_g, est_g] = BM3D(orig(:, :, 2), noisy(:, :, 2));
    [PSNR_b, est_b] = BM3D(orig(:, :, 3), noisy(:, :, 3));

    figure();
    subplot(1, 2, 1);
    imshow(noisy);
    subplot(1, 2, 2);
    imshow(cat(3, est_r, est_g, est_b));
    imwrite(cat(3, est_r, est_g, est_b), fname);
else
    [PSNR_y, est_y] = BM3D(orig, noisy);
    figure();
    subplot(1, 2, 1);
    imshow(noisy);
    subplot(1, 2, 2);
    imshow(est_y);
    imwrite(est_y, fname);
end
