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

% Z-Axis points 'inwards' (away from viewer)
%
% [ u v ]' = [ x y ]' / z * focal + principal_point
points_camera = points_world(1:2,:) ./ repmat(points_world(3,:),2,1) .* intrinsics.focal_length + repmat(intrinsics.principal_point,1,N);

if nargout > 1
        
    % calculate Jacobian symbolically
    syms x y z real;
    J = jacobian(camera_project([x y z]', intrinsics), [x y z]);
    
    % convert symbolic expression into function handle
    func = matlabFunction(J, 'vars', [x y z]);
    
    % evaluate function for all points
    J_project = reshape(cell2mat(arrayfun(func, points_world(1,:), points_world(2,:), points_world(3,:), 'UniformOutput', false)), 2, 3, N);
    
    assert(all(size(J_project) == [2,3,N]));
end

end

