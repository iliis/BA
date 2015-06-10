function [ points_key, J_transform ] = camera_transform( points_current, T_keyframe_to_current )
% transforms (rotates & translates) 3D points in current frame to
% 3D points in original (keyframe) camera frame
% i.e. moves keyframe camera to current camera position
%
% in paper: T*x
%
% INPUT:
%
% points_current
%   [x y z]' * N points in new (current frame) oordinate system
%
% T_keyframe_to_current
%   [Tx Ty Tz alpha beta gamma]'
%   Transformation to apply to points_keyframe (first rotation, then
%   translation)
%
%
% OUTPUT:
%
% points_keyframe
%   [x y z]' * N points in original (keyframe/world) coordinate system
%
% J_transform
%   3 * 6 * N Jacobian matrix of camera_transform(), evaluated at points_current
%
%               dTx dTy dTz dTalpha dTbeta dTgamma
% dtransform_x [ .   .   .     .       .       .    ]
% dtransform_y [ .   .   .     .       .       .    ]
% dtransform_z [ .   .   .     .       .       .    ]
%

N = size(points_current, 2);

% check parameters
assert(all(size(points_current) == [3 N]));
assert(numel(T_keyframe_to_current) == 6);
T_keyframe_to_current = reshape(T_keyframe_to_current, 6, 1); % ensure T is a colum vector

% apply transformation R * X + T
% points_current is list of column vectors
points_key = (angle2dcm(T_keyframe_to_current(4:6)) * points_current) + repmat(T_keyframe_to_current(1:3), 1, N);


if nargout > 1
        
    % calculate Jacobian symbolically
    syms x y z real;
    syms    Tx Ty Tz Talpha Tbeta Tgamma real;
    Tsym = [Tx Ty Tz Talpha Tbeta Tgamma];
    
    J = jacobian(camera_transform([x y z]', Tsym), Tsym);
    
    % evaluate symbolic derivation for current transformation -> still symbolic!
    J = subs(J, Tsym, T_keyframe_to_current');
    
    % convert symbolic expression into function handle
    func = matlabFunction(J, 'vars', [x y z]);
    
    % evaluate function for all points
    J_transform = reshape(cell2mat(arrayfun(func, points_current(1,:), points_current(2,:), points_current(3,:), 'UniformOutput', false)), 3, 6, N);
        
    assert(all(size(J_transform) == [3,6,N]));
end

end

