function D = read_depth_image(path, nr)
% output: N x 3 coordinates (U, V are in camera plane, D is distance from
% camera origin)

global_parameters

D = imread(fullfile(path, sprintf('depth%04u.png', nr))); % BW, 16bit

% reverse gamma correction
%D = imadjust(D,[],[],2.2);

% scale depth
D = double(D) / DEPTH_IMAGE_MAXVAL * (CAMERA_CLIPPING(2)-CAMERA_CLIPPING(1)) + CAMERA_CLIPPING(1);

end