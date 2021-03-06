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
assert(all(size(points_camera) == [2,N]), 'points must be 2 x N matrix');
assert(all(size(depths) == [1,N]), 'depths must be 1 x N row vector');
assert(isa(intrinsics, 'CameraIntrinsics'));

% project into world coordinates
% [x,y]' = ([u v]' - principal_point) * depth / focal_length
%    z   = depth
points_world = [ ...
    (points_camera(1,:) - intrinsics.principal_point(1)) .* depths ./ intrinsics.focal_length;
    (points_camera(2,:) - intrinsics.principal_point(2)) .* depths ./ intrinsics.focal_length;
    depths ]; % our Z-Axis points 'inwards' (away from viewer) => positive depth = positive Z


if nargout > 1
    
    % This code is actually never used ;)
    
    % calculate Jacobian symbolically
    syms u v D real;
    J = jacobian(camera_project_inverse([u v]', D, intrinsics), [u v D]);
    
    % convert symbolic expression into function handle
    func = matlabFunction(J, 'vars', [u v D]);
    
    % evaluate function for all pixels
    J_project_inv = reshape(cell2mat(arrayfun(func, points_camera(1,:), points_camera(2,:), depths, 'UniformOutput', false)), 3, 3, N);
    
    assert(all(size(J_project_inv) == [3,3,N]));
end

end

