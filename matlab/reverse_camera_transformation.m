function XYZ = reverse_camera_transformation(XYZ, translation, rotation)

% apply camera transformation

rot_matrix = quat2dcm(rotation)'; % inverse of rotation matrix is just transpose
for y = 1:size(XYZ,1)
    for x = 1:size(XYZ,2)
        XYZ(y,x,:) = shiftdim(rot_matrix * (shiftdim(XYZ(y,x,:)) - translation'), 1);
    end
end

end