function main_3DNR()

clear;
clc;
close all;

[filename, pathname, filterindex] = uigetfile( ...
{  '*.bmp','Bitmap files (*.bmp)'; ...
   '*.png','PNG files (*.png)'; ...
   '*.jpg','JPEG files (*.jpg)'; ...
   '*.*',  'All Files (*.*)'}, ...
   'Pick a file', ...
   'MultiSelect', 'on');

 if (iscell(filename) == 0);
     if filename == 0;
         number_of_images = 0;
     else
         number_of_images = 1;
     end
 else
     number_of_images = length(filename);
 end

fullfilename = fullfile(pathname, filename);
if number_of_images == 1
    [pathstr, name, ext] = fileparts(fullfilename);
else
    [pathstr, name, ext] = fileparts(fullfilename{number_of_images});
end

sum_frame = 0;
if number_of_images > 1
    for i = 1: number_of_images
        reference{i} = im2double(imread(fullfile(pathname, filename{i})));
        sum_frame = sum_frame + reference{i};
    end
    observed = reference{number_of_images};
    average_rgb_frame = sum_frame / number_of_images;
else
    reference{1} = im2double(imread(fullfile(pathname, filename)));
    observed = reference{1};
    average_rgb_frame = reference{1};
end

[image_height, image_width, channel] = size(observed);

fname = [pathstr '/' name '_' mat2str(number_of_images) '_frames_averaged.bmp'];
imwrite(average_rgb_frame, fname);

%
% blocks from referenced frame
%
ref_block_x_radius = 1;
ref_block_y_radius = 1;

ref_block_x_count = 2 * ref_block_x_radius + 1;
ref_block_y_count = 2 * ref_block_y_radius + 1;

ref_block_width = 4;
ref_block_height = 4;

block_x_count = ceil(image_width / ref_block_width);
block_y_count = ceil(image_height / ref_block_height);

for ref_y_block = 0 : ref_block_y_count - 1  
    for ref_x_block = 1 : ref_block_x_count  
        ref_block_x_index{ref_x_block + ref_y_block * ref_block_x_count} = (ref_x_block - 1 - ref_block_x_radius);    
        ref_block_y_index{ref_x_block + ref_y_block * ref_block_x_count} = (ref_y_block - ref_block_y_radius);  
    end
end

for image_y_start = 1 : ref_block_height : image_height - ref_block_height + 1
    for image_x_start = 1 : ref_block_width : image_width - ref_block_width + 1

        block_x_index = ceil(image_x_start / ref_block_width);
        block_y_index = ceil(image_y_start / ref_block_height);

        for ref_y_block = 0 : ref_block_y_count - 1
            for ref_x_block = 1 : ref_block_x_count
                block_x_start{ref_x_block + ref_y_block * ref_block_x_count} = image_x_start + ref_block_width * ref_block_x_index{ref_x_block + ref_y_block * ref_block_x_count}; 
                block_y_start{ref_x_block + ref_y_block * ref_block_x_count} = image_y_start + ref_block_height * ref_block_y_index{ref_x_block + ref_y_block * ref_block_x_count};
            end
        end

        for block_index = 1 :  ref_block_x_count * ref_block_y_count
            if (block_x_start{block_index} < 1)
                block_x_start{block_index} = 1;
            end
            if (block_y_start{block_index} < 1)
                block_y_start{block_index} = 1;
            end
            if (image_width - block_x_start{block_index} <  ref_block_width - 1)
                block_x_start{block_index} = image_width - ref_block_width + 1;
            end
            if (image_height - block_y_start{block_index} < ref_block_height - 1)
                block_y_start{block_index} = image_height - ref_block_height + 1;
            end
        end

        for image_index = 1 : number_of_images
%             figure();
            for block_index = 1 : ref_block_x_count * ref_block_y_count
                reference_blocks{image_index}{block_index} = reference{image_index}(block_y_start{block_index} : block_y_start{block_index} + ref_block_height - 1, block_x_start{block_index} : block_x_start{block_index} + ref_block_width - 1, :);
                if (image_index == number_of_images) && (block_index == ref_block_y_radius * ref_block_x_count + ref_block_x_radius + 1)
                   current_block =  reference_blocks{image_index}{block_index};
                end
%                 subplot(ref_block_x_count, ref_block_y_count, block_index);
%                 imshow(reference_blocks{image_index}{block_index});
            end
        end

        res_block = weighted_average(current_block, reference_blocks);

        restored(image_y_start : image_y_start + ref_block_height - 1, image_x_start : image_x_start + ref_block_width - 1, :) = res_block;
    end
end


figure();

subplot(2, 2, 1);
imshow(average_rgb_frame);
title(['Averaged image,' mat2str(number_of_images) ' number of images']);
subplot(2, 2, 2);
imshow(observed);
title('orignal image');

subplot(2, 2, 3);
imshow(restored);
title('3D denoise image');
fname_restored = [pathstr '/' name '_' mat2str(number_of_images) '_frames_restored.bmp'];
imwrite(restored, fname_restored);

subplot(2, 2, 4);
imshow(reference_blocks{number_of_images}{ref_block_x_count * ref_block_y_count});

end


function res_block = weighted_average(cur_block, ref_blocks)

    image_count = length(ref_blocks);
    block_count = length(ref_blocks{1});
    weight_sum = 0;
    [block_height, block_width, channel] = size(cur_block);
    res_block = zeros(block_height, block_width, channel);
    
    for image_index = 1 : image_count
        for block_index = 1: block_count
            dist = sum(sum( (ref_blocks{image_index}{block_index} - cur_block) .* (ref_blocks{image_index}{block_index} - cur_block) ));
            MAG = (block_height * block_width) * ref_blocks{image_index}{block_index}(ceil(block_height/2), ceil(block_width/2)) - sum(sum(ref_blocks{image_index}{block_index}));
            MAG = abs(MAG) / (block_height * block_width - 1);
            
            if MAG < 0.5
                alpha = 1;
            else
                alpha = 2;
            end
            weight = exp(-3.0 * dist(1, 1, :) * alpha);
            weight_sum = weight_sum + weight;

            res_block(:, :, 1) = res_block(:, :, 1) + ref_blocks{image_index}{block_index}(:, :, 1) .* weight(:, :, 1);
            res_block(:, :, 2) = res_block(:, :, 2) + ref_blocks{image_index}{block_index}(:, :, 2) .* weight(:, :, 2);
            res_block(:, :, 3) = res_block(:, :, 3) + ref_blocks{image_index}{block_index}(:, :, 3) .* weight(:, :, 3);
        end
    end
    
    res_block(:, :, 1) = res_block(:, :, 1) ./ weight_sum(:, :, 1);
    res_block(:, :, 2) = res_block(:, :, 2) ./ weight_sum(:, :, 2);
    res_block(:, :, 3) = res_block(:, :, 3) ./ weight_sum(:, :, 3);
    
end





