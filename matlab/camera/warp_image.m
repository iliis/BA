function warped = warp_image(depth, colors, T)

%disp(['warping image with pos = [' num2str(position) '] and rotation = [' num2str(rotation) ']']);

XYZ = project_to_space(depth);
XYZ = apply_camera_transformation(XYZ, T);
warped = project_to_camera(XYZ, colors);

end