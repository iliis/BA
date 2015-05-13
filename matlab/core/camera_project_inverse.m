function [ points_world, J_project_inv ] = camera_project_inverse( points_camera, depths, intrinsics )
%CAMERA_PROJECT_INVERSE projects 2D points on camera into 3D world
%
% in paper: pi^-1(x, D(x))
%
% INPUT:
%
% points_camera
%   N * [u v] points on camera sensor (in pixels) 
%
% depths
%   N depth values (units?!), corresponding to pixels in points_camera
%
% intrinsics
%   instance of camera_intrinsics
%   contains size of image sensor, focal length, principal point etc.
%
% OUTPUT:
%
% points_world
%   N * [x y z] points in world coordinates (usually in keyframe)
%
% J_proj_inv
%   3 * 3 * N Jacobian of camera_project_inverse
%
%                du dv dD
%   dproj_inv_x [ .  .  . ]
%   dproj_inv_y [ .  .  . ]
%   dproj_inv_z [ .  .  . ]
%

N = size(points_camera,1);

% check parameters
assert(all(size(points_camera) == [N,2]));
assert(all(size(depths) == [N,1]));
assert(isa(intrinsics, 'CameraIntrinsics'));

% project into world coordinates
points_world = [ ...
    (points_camera - repmat(intrinsics.principal_point,N,1)) .* repmat(depths,1,2) ./ intrinsics.focal_length ...
    -depths ]; % our Z-Axis points 'outwards' (toward viewer) => positive depth = negative Z

end

