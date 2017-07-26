%   denoising_dwt.m - image denoise using wavelet
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

function y = denoising_dwt(x, wname, tname, tuning)
% Local Adaptive Image Denoising Algorithm
% Usage :
%        y = denoising_dwt(x)
% INPUT :
%        x - a noisy image
% OUTPUT :
%        y - the corresponding denoised image

if nargin < 2
    wname = 'haar';
    tname = 'penalhi';
    tuning = 1;
end

glb = 0;

alpha = 1;

level = 5;

% wavedec2 is a two-dimensional wavelet analysis function.
% Vector C is organized as a vector with A(N), H(N), V(N), D(N), H(N-1), V(N-1), D(N-1), ..., H(1), V(1), D(1), where A, H, V, and D are each a row vector. Each vector is the vector column-wise storage of a matrix.
% 
% A contains the approximation coefficients
% H contains the horizontal detail coefficients
% V contains the vertical detail coefficients
% D contains the diagonal detail coefficients
% Matrix S is such that
% 
% S(1,:) = size of approximation coefficients(N).
% S(i,:) = size of detail coefficients(N-i+2) for i = 2, ...N+1 and S(N+2,:) = size(X).
%
[C, S] = wavedec2(x, level, wname);

sorh = 's';

if glb == 1
% Estimate the noise standard deviation from the
% detail coefficients at level 1.
    det1 = detcoef2('compact', C, S, 1);
    sigma = median(abs(det1))/0.6745;
    keepapp = 1;

% Use wbmpen for selecting global threshold  
% for image de-noising.
    glb_thr = wbmpen(C, S, sigma, alpha)
    
    XDEN = wdencmp('gbl', C, S, wname, level, glb_thr, sorh, keepapp);
else
% Obtain denoising (wavelet shrinkage) thresholds. Use the Birge-Massart strategy with a tuning parameter of 3.

% De-noising using level dependent thresholds.
% [C,S] is the wavelet decomposition structure of the image to be de-noised,
%  SCAL defines the multiplicative threshold rescaling (see wden for more information) and
%  ALFA is a sparsity parameter (see wbmpen for more information).
% 
% THR = wthrmngr('dw2ddenoLVL','penalhi',C,S,ALFA) 
%       ALFA must be such that 2.5 < ALFA < 10
% THR = wthrmngr('dw2ddenoLVL','penalme',C,S,ALFA) 
%       ALFA must be such that 1.5 < ALFA < 2.5
% THR = wthrmngr('dw2ddenoLVL','penallo',C,S,ALFA) 
%       ALFA must be such that 1 < ALFA < 2
% THR = wthrmngr('dw2ddenoLVL','sqtwolog',C,S,SCAL)
% THR = wthrmngr('dw2ddenoLVL','sqrtbal_sn',C,S)
%
    if strcmp(tname, 'penalhi')
        alpha = 5;
    elseif strcmp(tname, 'penalme')
        alpha = 2;
    elseif strcmp(tname, 'penallo')
        alpha = 1.5;
    elseif strcmp(tname, 'sqtwolog')
        alpha = 1;
    end

    alpha = alpha * tuning;
    if strcmp(tname, 'sqrtbal_sn')
        lvd_thr = wthrmngr('dw2ddenoLVL', tname, C, S)
    else
        lvd_thr = wthrmngr('dw2ddenoLVL', tname, C, S, alpha)
    end

% wdencmp is a one- or two-dimensional de-noising and compression-oriented function.
% wdencmp performs a de-noising or compression process of a signal or an image, using wavelets.
% 
% [XC,CXC,LXC,PERF0,PERFL2] = wdencmp('gbl',X,'wname',N,THR,SORH,KEEPAPP) returns a de-noised or compressed version XC of input signal X (one- or two-dimensional) obtained by wavelet coefficients thresholding using global positive threshold THR.
% 
% Additional output arguments [CXC,LXC] are the wavelet decomposition structure of XC (see wavedec or wavedec2 for more information). PERF0 and PERFL2 are L2 -norm recovery and compression score in percentage.
% 
% PERFL2 = 100 * (vector-norm of CXC / vector-norm of C)2 if [C,L] denotes the wavelet decomposition structure of X.
%
    [XDEN, cfsDEN, dimCFS] = wdencmp('lvd', C, S, wname, level, lvd_thr, sorh);
end

% figure;
% subplot(1,2,1);
% imagesc(x); colormap gray; axis off;
% title('Noisy Image');
% subplot(1,2,2);
% imagesc(XDEN); colormap gray; axis off;
% title('Denoised Image');

y = XDEN;

end
