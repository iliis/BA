% basic test for new core functions
% loads an image pair and warps the keyframe onto the current frame

image_path = 'input/testscene1';
T = [-2 0 4 0 0 0]'; % TODO: read this from camera_trajectory.csv
%T = [0 0 0 0 0 0]';

D1 = read_depth_image(image_path, 1);
I1 = read_intensity_image(image_path, 1);

D2 = read_depth_image(image_path, 2);
I2 = read_intensity_image(image_path, 2);

% Blender uses 35mm focal length and a 32mm wide sensor
FOCAL = 0.035 * size(I1,2) / 0.032;

intrinsics = CameraIntrinsics(size(I1,2), size(I1,1), FOCAL);

intensities1 = image_to_list(I1);
depths1      = image_to_list(D1);

[X,Y] = meshgrid(1:size(I1,2), 1:size(I1,1));

points_keyframe_camera = image_to_list(cat(3,X,Y));

points_world          = camera_project_inverse(points_keyframe_camera, depths1, intrinsics);
points_current        = camera_transform(points_world, T);
points_current_camera = camera_project(points_current, intrinsics);

% filter points outside of image range
valid_points          = is_in_img_range(points_current_camera, I2);
points_current_camera = points_current_camera(:, valid_points);
intensities2          = camera_intensity_sample(points_current_camera, I2);

% plot keyframe points in current frame
%scatter(points_current_camera(:,1), points_current_camera(:,2), 2, intensities2);
%colormap('gray');
%hold on; plot([1 size(I1,2) size(I1,2) 1 1], [1 1 size(I1,1) size(I1,1) 1], '-'); hold off;
%set(gca, 'Ydir', 'reverse'); % 0,0 is at the top left

% only compare points that projected inside the current image
intensities1 = intensities1(:, valid_points);
err = (intensities1 - intensities2).^2;

%figure;
subplot(2,1,1);
colormap('jet');
scatter(points_keyframe_camera(1,valid_points), points_keyframe_camera(2,valid_points), 2, err);
colorbar();
title('errors in original (keyframe) view');
subplot(2,1,2);
scatter(points_current_camera(1,:), points_current_camera(2,:), 2, err);
colorbar();
title('errors of warped points (in current view)');

%points_cc_valid = points_current_camera(is_in_img_range(points_current_camera, I2), :);
