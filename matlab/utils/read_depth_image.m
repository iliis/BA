function D = read_depth_image(path, nr)
% output: H * W depth values (distance from camera origin)

% depth images are 16bit grayscale, color images 8bit RGB
% TODO: move this into a global object or something?
DEPTH_IMAGE_MAXVAL = 65535;
CAMERA_CLIPPING = [0.1 100];

D = imread(fullfile(path, sprintf('depth%04u.png', nr))); % BW, 16bit

% reverse gamma correction
%D = imadjust(D,[],[],2.2);

% scale depth
D = double(D) / DEPTH_IMAGE_MAXVAL * (CAMERA_CLIPPING(2)-CAMERA_CLIPPING(1)) + CAMERA_CLIPPING(1);

end