function [ errors, J_warp ] = camera_warp( scene, T, plot )
% projects points into world, transforms and projects back
%
% TODO: write some tests for this function! There are some critical things
% with the inverted V-axis...
%
% INPUT:
%
% scene, containing:
% (TODO: update this, e.g. add a get_keyframe(i) or something to Scene class)
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
% 


image_keyframe  = scene.I1;
depths_keyframe = scene.D1;
image_current   = scene.I2;

[H, W] = size(image_keyframe);

calc_jacobian = nargout > 1;
show_plots    = nargin > 2;

% check input parameters
assert(numel(T) == 6);
assert(isa(scene, 'Scene'));


[X,Y] = meshgrid(1:W, 1:H);

points_keyframe_camera = image_to_list(cat(3,X,Y));

points_world = camera_project_inverse(points_keyframe_camera, image_to_list(depths_keyframe), scene.intrinsics);

if calc_jacobian
    [points_current,        J_T] = camera_transform(points_world, T);
    [points_current_camera, J_P] = camera_project(points_current, scene.intrinsics);
else
    points_current        = camera_transform(points_world, T);
    points_current_camera = camera_project(points_current, scene.intrinsics);
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
errors = -(intensities_keyframe - intensities_current);

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

% TODO: move this elsewhere ...
if show_plots
    whitebg([0 0 0]);
    
    subplot(2,3,1);
    colormap('jet');
    scatter(points_keyframe_camera(1,valid_points), points_keyframe_camera(2,valid_points), 2, errors.^2, 'filled');
    set(gca,'YDir','reverse'); xlim([0 W]); ylim([0 H]);
    colorbar();
    title('errors in original (keyframe) view');

    subplot(2,3,2);
    scatter(points_keyframe_camera(1,valid_points), points_keyframe_camera(2,valid_points), 2, intensities_keyframe, 'filled');
    set(gca,'YDir','reverse'); xlim([0 W]); ylim([0 H]);
    colorbar();
    title('keyframe intensity image');
    
    subplot(2,3,3);
    imagesc(image_keyframe);
    title('keyframe image');

    subplot(2,3,4);
    scatter(points_current_camera(1,:), points_current_camera(2,:), 2, errors.^2, 'filled');
    set(gca,'YDir','reverse'); xlim([0 W]); ylim([0 H]);
    colorbar();
    title('errors of warped points (in current view)');

    subplot(2,3,5);
    scatter(points_current_camera(1,:), points_current_camera(2,:), 2, intensities_keyframe, 'filled');
    set(gca,'YDir','reverse'); xlim([0 W]); ylim([0 H]);
    colorbar();
    title('warped keyframe intensity image');
    
    subplot(2,3,6);
    imagesc(image_current);
    title('current image');
end

end

