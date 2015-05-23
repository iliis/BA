% basic test for new core functions
% loads an image pair and warps the keyframe onto the current frame

image_path = 'input/testscene1';
T = [2 0 -4 0 0 0]'; % TODO: read this from camera_trajectory.csv
%T = [0 0 0 0 0 0]';

D1 = read_depth_image(image_path, 1);
I1 = read_intensity_image(image_path, 1);

D2 = read_depth_image(image_path, 2);
I2 = read_intensity_image(image_path, 2);

% Blender uses 35mm focal length and a 32mm wide sensor
FOCAL = 0.035 * size(I1,2) / 0.032;

intrinsics = CameraIntrinsics(size(I1,2), size(I1,1), FOCAL);

% do the actual calculations
err = camera_warp(I1,D1,I2,T,intrinsics);

%figure;

disp(['total error: ' num2str(sqrt(sum(err)))]);

%points_cc_valid = points_current_camera(is_in_img_range(points_current_camera, I2), :);
