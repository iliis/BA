function warped = warp_image(depth, colors, position, rotation)

XYZ = project_to_space(depth);
XYZ = apply_camera_transformation(XYZ, position, rotation);
warped = project_to_camera(XYZ, colors);

end