function RGB = read_color_image(path, nr)
% output: H x W x 3 list of RGB values in range 0 to 1

% depth images are 16bit grayscale, color images 8bit RGB
COLOR_IMAGE_MAXVAL = 255;

RGB = imread(fullfile(path, sprintf('color%04u.png', nr))); % RGB, 8bit / channel

RGB = double(RGB) ./ COLOR_IMAGE_MAXVAL;

end