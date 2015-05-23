function [ points_world, J_project_inv ] = camera_project_inverse( points_camera, depths, intrinsics )
%CAMERA_PROJECT_INVERSE projects 2D points on camera into 3D world
%
% in paper: pi^-1(x, D(x))
%
% INPUT:
%
% points_camera
%   [u v]' * N points on camera sensor (in pixels) 
%
% depths
%   N depth values (units?!), corresponding to pixels in points_camera
%   (row vector!)
%
% intrinsics
%   instance of camera_intrinsics
%   contains size of image sensor, focal length, principal point etc.
%
% OUTPUT:
%
% points_world
%   [x y z]' * N points in world coordinates (usually in keyframe)
%
% J_proj_inv
%   3 * 3 * N Jacobian of camera_project_inverse
%
%                du dv dD
%   dproj_inv_x [ .  .  . ]
%   dproj_inv_y [ .  .  . ]
%   dproj_inv_z [ .  .  . ]
%

N = size(points_camera,2);

% check parameters
assert(all(size(points_camera) == [2,N]));
assert(all(size(depths) == [1,N]));
assert(isa(intrinsics, 'CameraIntrinsics'));

% project into world coordinates
% [x,y]' = ([u v]' - principal_point) * depth / focal_length
%    z   = -depth
points_world = [ ...
    (points_camera - repmat(intrinsics.principal_point,1,N)) .* repmat(depths,2,1) ./ intrinsics.focal_length; ...
    -depths ]; % our Z-Axis points 'outwards' (toward viewer) => positive depth = negative Z

end

