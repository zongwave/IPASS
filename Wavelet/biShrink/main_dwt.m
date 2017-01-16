function [PSNR, est] = main_dwt(orig, noisy)

% Main function
% Usage :
%        main
% INPUT :
%        Raw Lena image
% OUTPUT :
%        PSNR value of the denoised image
%
% Load clean image

if (exist('orig') ~= 1) & (exist('noisy') ~= 1)
    fid = fopen('barbara','r');
    %fid = fopen('boat','r');
    orig  = fread(fid,[512 512],'unsigned char');
    orig = orig';
    fclose(fid)
    orig = imread('Lena512.bmp','bmp');
    orig = double(orig);
    N = 512;

    [row, col, channel] = size(orig);

    % Noise variance
    if (exist('sigma') ~= 1),
        sigma = 60; %% default standard deviation of the AWGN
    end

    if channel == 3
        n(:, :, 1) = sigma*randn(N);
        n(:, :, 2) = sigma*randn(N);
        n(:, :, 3) = sigma*randn(N);
    else
        n = sigma*randn(N);
    end

    % Add noise
    noisy = orig + n;
end

[row, col, channel] = size(noisy);

% Run local adaptive image denoising algorithm
if channel == 3
    est(:, :, 1) = denoising_dwt(noisy(:, :, 1));
    est(:, :, 2) = denoising_dwt(noisy(:, :, 2));
    est(:, :, 3) = denoising_dwt(noisy(:, :, 3));
else
    est = denoising_dwt(noisy);
end

figure();
subplot(1, 2, 1);
imshow(im2double(noisy) / 255.0);
subplot(1, 2, 2);
imshow(im2double(est) / 255.0);

% Calculate the error
if (exist('orig') == 1)
    err = orig - est;

    % Calculate the PSNR value
    PSNR = 20*log10(256/std(err(:)))
end

end
