function [ intensities, J_intensity ] = camera_intensity_sample( points_camera, image )
% samples camera image
%
% in paper: I(x)
%
%
% INPUT:
%
% points_camera
%   [u v]' * N points in camera coordinate system (in pixels)
%       where:
%           [1 1]' = bottom left pixel
%           [W H]' = top right pixel
%   see CameraIntrinsics for details
%
% image
%   H * W intensity image (i.e. greyscale)
%
%
% OUTPUT:
%
% intensities
%   N intensity values (row vector)
%
% J_intensity
%   1 * 2 * N Jacobian of camera_intensity_sample()
%
%   [  dI(u,v)/du  dI(u,v)/dv  ]

N = size(points_camera,2);
[H, W] = size(image);

% check parameters
assert(all(size(points_camera) == [2, N]));
assert(H > 0); assert(W > 0);
assert(all(all(points_camera >= 1)));
assert(all(all(points_camera(1,:) <= W)));
assert(all(all(points_camera(2,:) <= H)));

% [1,1] is bottom left in image coordinates -> invert V-axis!
image = flipud(image);

% actually sample image ;)
intensities = interp2(image, points_camera(1,:), points_camera(2,:));


if nargout > 1
    
    % approximate intensity differential with finite differences
    
    % pad with zeros for sampling to work precisely
    diff_x = [zeros(H,1), diff(image, 1, 2), zeros(H,1)];
    diff_y = [zeros(1,W); diff(image, 1, 1); zeros(1,W)];
    
    % sample differences
    J_intensity_x = interp2(diff_x, points_camera(1,:) + 0.5, points_camera(2,:));
    J_intensity_y = interp2(diff_y, points_camera(1,:),       points_camera(2,:) + 0.5);
    
    J_intensity = [ permute(J_intensity_x, [1,3,2]), permute(J_intensity_y, [1,3,2]) ];
    
    
    % other methods one could try to calculate the image gradient:
    
    %[diff_x, diff_y] = imgradientxy(image, 'IntermediateDifference');
    
    %diff_x = conv2(image, [-1 0 1]/2, 'same');
    %diff_y = conv2(image, [-1 0 1]'/2, 'same');
end

end

