function [ points_camera, J_project ] = camera_project( points_world, intrinsics )
%CAMERA_PROJECT projects points from 3D space (world) on to camera screen
%
% in paper: pi(x)
%
%
% INPUT:
%
% points_world
%   [x y z]' * N points in world coordinates (usually in current frame)
%
% intrinsics
%   instance of camera_intrinsics
%   contains size of image sensor, focal length, principal point etc.
%
%
% OUTPUT:
%
% points_camera
%   [u v]' * N points on camera sensor (in pixels)
%
% J_project
%   2 * 3 * N Jacobian of camera_project()
%
%            dx dy dz
%   dproj_u [ .  .  . ]
%   dproj_v [ .  .  . ]

N = size(points_world,2);

% check parameters
assert(all(size(points_world) == [3,N]));
assert(isa(intrinsics, 'CameraIntrinsics'));

% project back onto image plane



% X-Axis is mirrored relative to U-Axis in camera plane
% --> U = -X / Z * focal + principal
points_world(1,:) = -points_world(1,:);

% Z-Axis points 'inwards' (away from viewer)
%
% [ u v ]' = [ x y ]' / z * focal + printcipal_point
points_camera = points_world(1:2,:) ./ repmat(points_world(3,:),2,1) .* intrinsics.focal_length + repmat(intrinsics.principal_point,1,N);

end

