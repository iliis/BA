% start afresh
clear all;

imgnr = 1;

% initial position of camera
camera_pos = [ 7.5 -6.5 5.3 ];
%camera_rot = [ 0.782 0.482 0.213 0.334 ];
camera_rot = [0.7816000580787659, 0.48170700669288635, 0.21292176842689514, 0.3342514932155609];


UVD = read_depth_image(imgnr);
XYZ = project_to_space(UVD, camera_pos, camera_rot);

colors = read_color_image(imgnr);

XYZ = apply_camera_transformation(XYZ, camera_pos, camera_rot);

XYZ = apply_camera_transformation(XYZ, [2 0 0], [1 0 0 0]);

XYZ = reverse_camera_transformation(XYZ, camera_pos, camera_rot);

%plot_pointcloud(XYZ, colors);
project_to_camera(XYZ, colors);

% lochkamera nachlesen
% erstmal Euler Winkel