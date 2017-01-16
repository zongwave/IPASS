function main_bfilter()

[filename, pathname] = uigetfile( ...
{  '*.bmp','Bitmap files (*.bmp)'; ...
   '*.png','PNG files (*.png)'; ...
   '*.jpg','JPEG files (*.jpg)'; ...
   '*.*',  'All Files (*.*)'}, ...
   'Pick a file');

imfilename = fullfile(pathname, filename);

I=im2double(imread(imfilename));
w     = 15;       % bilateral filter half-width
sigma = [3 0.1]; % bilateral filter standard deviations

I_bi=bfilter(I,w,sigma);

figure();
subplot(1,2,1);
imshow(I);
subplot(1,2,2);
imshow(I_bi)

imwrite(I_bi, [imfilename '_' mat2str(w) '_bilater.bmp']);

% I1=bilateralFilter_fast(I(:,:,1));
% I2=bilateralFilter_fast(I(:,:,2));
% I3=bilateralFilter_fast(I(:,:,3));
