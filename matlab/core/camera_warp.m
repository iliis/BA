function [ errors, J_warp ] = camera_warp( image_keyframe, depths_keyframe, image_current, T, intrinsics, plot )
% projects points into world, transforms and projects back
%
% TODO: write some tests for this function! There are some critical things
% with the inverted V-axis...
%
% INPUT:
%
% image_keyframe:
%   H * W intensity image, this will be warped
%
% depths_keyframe:
%   H * W depths of keyframe image
%
% image_current:
%   H * W intensity image, this is the target
%
% T:
%   [x y z alpha beta gamma]' transformation to apply
%
% intrinsics:
%   camera intrinsics
% 

[H, W] = size(image_keyframe);

calc_jacobian = nargout > 1;
show_plots = nargin > 5;

% check input parameters
assert(all(size(depths_keyframe) == [H,W]), 'depth matrix must have same size as intensity image');
assert(all(size(image_current)   == [H,W]), 'current image must have same size as keyframe image');
assert(numel(T) == 6);
assert(isa(intrinsics, 'CameraIntrinsics'));


[X,Y] = meshgrid(1:W, 1:H);

points_keyframe_camera = image_to_list(cat(3,X,Y));

% attention: inverted V-Axis -> flip depths!
points_world = camera_project_inverse(points_keyframe_camera, image_to_list(depths_keyframe), intrinsics);

if calc_jacobian
    [points_current,        J_T] = camera_transform(points_world, T);
    [points_current_camera, J_P] = camera_project(points_current, intrinsics);
else
    points_current        = camera_transform(points_world, T);
    points_current_camera = camera_project(points_current, intrinsics);
end



% filter points outside of image range
valid_points              = is_in_img_range(points_current_camera, image_current);
points_current_camera     = points_current_camera(:, valid_points);

if calc_jacobian
    J_T = J_T(:,:,valid_points);
    J_P = J_P(:,:,valid_points);
    [intensities_current, J_I] = camera_intensity_sample(points_current_camera, image_current);
else
    intensities_current = camera_intensity_sample(points_current_camera, image_current);
end

% only compare points that projected inside the current image
intensities_keyframe = camera_intensity_sample(points_keyframe_camera, image_keyframe);
intensities_keyframe = intensities_keyframe(:, valid_points);
errors = (intensities_keyframe - intensities_current);

if calc_jacobian
    N = size(points_current_camera,2);
    assert(N == size(J_T,3));
    assert(N == size(J_P,3));
    assert(N == size(J_I,3));
    
    %J_warp = z_matmultiply(J_I, z_matmultiply(J_P, J_T)); % 3.121s
    %J_warp = z_matmultiply(z_matmultiply(J_I, J_P), J_T); % 6.039s
    
    J_warp = zeros(N,numel(T));
    % this seems to be one of the fastest way of doing N matrix multiplications:
    for i = 1:N
        J_warp(i,:) = J_I(:,:,i) * J_P(:,:,i) * J_T(:,:,i);
    end
    
    assert(all(size(J_warp) == [N,6]));
end

if show_plots
    subplot(2,2,1);
    colormap('jet');
    scatter(points_keyframe_camera(1,valid_points), points_keyframe_camera(2,valid_points), 2, errors);
    colorbar();
    title('errors in original (keyframe) view');

    subplot(2,2,2);
    scatter(points_keyframe_camera(1,valid_points), points_keyframe_camera(2,valid_points), 2, intensities_keyframe);
    colorbar();
    title('keyframe intensity image');

    subplot(2,2,3);
    scatter(points_current_camera(1,:), points_current_camera(2,:), 2, errors);
    colorbar();
    title('errors of warped points (in current view)');

    subplot(2,2,4);
    scatter(points_current_camera(1,:), points_current_camera(2,:), 2, intensities_keyframe);
    colorbar();
    title('warped keyframe intensity image');
end

end

