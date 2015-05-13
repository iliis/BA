function [ intensities, J_intensity ] = camera_intensity_sample( points_camera, image )
% samples camera image
%
% in paper: I(x)
%
%
% INPUT:
%
% points_camera
%   N * [u v] points in camera coordinate system (in pixels)
%   [1 1] = top left pixel
%   [W H] = bottom right pixel
%   see CameraIntrinsics for details
%
% image
%   H * W intensity image (i.e. greyscale)
%
%
% OUTPUT:
%
% intensities
%   N intensity values
%
% J_intensity
%   1 * 2 * N Jacobian of camera_intensity_sample()
%
%   [  dI(u,v)/du  dI(u,v)/du  ]

N = size(points_camera,1);
[H, W] = size(image);

% check parameters
assert(all(size(points_camera) == [N, 2]));
assert(H > 0); assert(W > 0);
assert(all(all(points_camera >= 1)));
assert(all(all(points_camera(:,1) <= W)));
assert(all(all(points_camera(:,2) <= H)));

% actually sample image ;)
intensities = interp2(image, points_camera(:,1), points_camera(:,2));

end

