function XYZ = project_to_space(UVD, camera_pos, camera_rot)
% input:  N x 3 list of points in the camera plane
%         3D coordinates of camera and 4D quaternion of camera orientation
% output: N x 3 list of points in real space

% properties of camera (in Blender Units / meter)
focal = 0.035; % 35mm

% Blender's camera has Y as 'up' and -Z as 'into the scene'
%XYZ =  [UVD(:,1) -UVD(:,2) -focal*ones(size(UVD,1),1)] .* repmat(UVD(:,3), 1, 3);
U = UVD(:,1);
V = UVD(:,2);
D = UVD(:,3);
pointsx = U.*D;
pointsy = -V.*D;
pointsz = -focal.*D;

XYZ = [pointsx'; pointsy'; pointsz'];

% apply camera transformation
XYZ = (quat2dcm(camera_rot) * XYZ)'; % + repmat(camera_pos, size(XYZ,1), 1);

end