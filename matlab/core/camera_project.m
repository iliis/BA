function [ points_camera, J_project ] = camera_project( points_world, intrinsics )
%CAMERA_PROJECT projects points from 3D space (world) on to camera screen
%
% in paper: pi(x)
%
%
% INPUT:
%
% points_world
%   N * [x y z] points in world coordinates (usually in current frame)
%
% intrinsics
%   instance of camera_intrinsics
%   contains size of image sensor, focal length, principal point etc.
%
%
% OUTPUT:
%
% points_camera
%   N * [u v] points on camera sensor (in pixels)
%
% J_project
%   2 * 3 * N Jacobian of camera_project()
%
%            dx dy dz
%   dproj_u [ .  .  . ]
%   dproj_v [ .  .  . ]

N = size(points_world,1);

% check parameters
assert(size(points_world) == [N,3]);
assert(isa(intrinsics, 'CameraIntrinsics'));

end

