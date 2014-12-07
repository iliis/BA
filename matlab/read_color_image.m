function RGB = read_color_image(nr)
% output: N x 3 list of RGB values in range 0 to 1

RGB = imread(sprintf('color%04u.png', nr)); % RGB, 8bit / channel

c = numel(RGB(:,:,1));
RGB = double([reshape(RGB(:,:,1), [c,1]) reshape(RGB(:,:,2), [c,1]) reshape(RGB(:,:,3), [c,1])]) ./ 255;

end