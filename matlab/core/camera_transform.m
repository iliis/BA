function [ points_current, J_transform ] = camera_transform( points_keyframe, T_keyframe_to_current )
% transforms (rotates & translates) 3D points in original (keyframe) frame to
% 3D points in current camera frame
%
% in paper: T*x
%
%
% INPUT:
%
% points_keyframe
%   N * [x y z] points in original (keyframe/world) coordinate system
%
% T_keyframe_to_current
%   [Tx Ty Tz alpha beta gamma]
%   Transformation to apply to points_keyframe (first rotation, then
%   translation)
%
%
% OUTPUT:
%
% points_current
%   N * [x y z] points in new (current frame) coordinate system
%
% J_transform
%   3 * 6 * N Jacobian matrix of camera_transform(), evaluated at points_keyframe
%
%               dTx dTy dTz dTalpha dTbeta dTgamma
% dtransform_x [ .   .   .     .       .       .    ]
% dtransform_y [ .   .   .     .       .       .    ]
% dtransform_z [ .   .   .     .       .       .    ]
%

N = size(points_keyframe, 1);

% check parameters
assert(size(points_keyframe) == [N 3]);
assert(size(T_keyframe_to_current) == [1 6]);


end

