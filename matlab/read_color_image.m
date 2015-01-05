function RGB = read_color_image(nr)
% output: H x W x 3 list of RGB values in range 0 to 1

global_parameters

RGB = imread(sprintf('color%04u.png', nr)); % RGB, 8bit / channel

%c = numel(RGB(:,:,1));
%RGB = double([reshape(RGB(:,:,1), [c,1]) reshape(RGB(:,:,2), [c,1]) reshape(RGB(:,:,3), [c,1])]) ./ 255;

RGB = double(RGB) ./ COLOR_IMAGE_MAXVAL;

end