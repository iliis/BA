function [ points_current, J_transform ] = camera_transform( points_keyframe, T_keyframe_to_current_inv )
% transforms (rotates & translates) 3D points in original (keyframe) frame to
% 3D points in current camera frame
%
% in paper: T*x
%
% TODO: T_keyframe_to_current is actually the other way around (we're
% moving the points, not the camera!) What is the convention here? E.g if I
% move the camera by 5m into positive X direction, Tx is -5m !
%
%
% INPUT:
%
% points_keyframe
%   [x y z]' * N points in original (keyframe/world) coordinate system
%
% T_keyframe_to_current
%   [Tx Ty Tz alpha beta gamma]'
%   Transformation to apply to points_keyframe (first rotation, then
%   translation)
%
%
% OUTPUT:
%
% points_current
%   [x y z]' * N points in new (current frame) coordinate system
%
% J_transform
%   3 * 6 * N Jacobian matrix of camera_transform(), evaluated at points_keyframe
%
%               dTx dTy dTz dTalpha dTbeta dTgamma
% dtransform_x [ .   .   .     .       .       .    ]
% dtransform_y [ .   .   .     .       .       .    ]
% dtransform_z [ .   .   .     .       .       .    ]
%

N = size(points_keyframe, 2);

% check parameters
assert(all(size(points_keyframe) == [3 N]));
assert(numel(T_keyframe_to_current_inv) == 6);

% apply transformation R * X + T
% points_keyframe is list of column vectors
points_current = (angle2dcm(T_keyframe_to_current_inv(4:6)) * points_keyframe) + repmat(T_keyframe_to_current_inv(1:3), 1, N);

end

