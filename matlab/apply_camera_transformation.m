function XYZ = apply_camera_transformation(XYZ, translation, rotation)
% input: XYZ = H x W x 3 array of points in 3D space
%        translation: 3 dimensional row vector
%        rotation: [W X Y Z] quaternion

W = size(XYZ,2);
H = size(XYZ,1);

% repackage points from 2D-array into list
points = image_to_list(XYZ);

% actualy do the transformation
points = (quat2dcm(rotation) * points')' + repmat(translation, W*H, 1);

% repackage points into 2D array of 3D points (i.e. 3 dimensional matrix)
XYZ = list_to_image(points, [H,W]);


% TODO: use something less naive here
%rot_matrix = quat2dcm(rotation);
%for y = 1:size(XYZ,1)
%    for x = 1:size(XYZ,2)
%        XYZ(y,x,:) = shiftdim(rot_matrix * shiftdim(XYZ(y,x,:)) + translation', 1);
%    end
%end

end