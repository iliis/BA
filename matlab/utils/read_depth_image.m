function D = read_depth_image(path, nr, intrinsics)
% output: H * W depth values (distance from camera origin)


assert(isa(intrinsics, 'CameraIntrinsics'));

% depth images are 8bit grayscale, color images 8bit RGB
% TODO: move this into a global object or something?
% DEPTH_IMAGE_MAXVAL = 65535;
% CAMERA_CLIPPING = [0.1 100];

D = imread(fullfile(path, sprintf('depth%04u.png', nr))); % BW, 8bit

% reverse gamma correction
%D = imadjust(D,[],[],2.2);

% scale depth
%D = double(D) / DEPTH_IMAGE_MAXVAL * (CAMERA_CLIPPING(2)-CAMERA_CLIPPING(1)) + CAMERA_CLIPPING(1);
D = double(D) / intrinsics.depth_img_depth * (intrinsics.far_clipping - intrinsics.near_clipping) + intrinsics.near_clipping;

end
