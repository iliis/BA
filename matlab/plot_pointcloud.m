function plot_pointcloud(XYZ, colors)
% XYZ:    H x W x 3 array of points in 3D space
% colors: H x W x 3 array of RGB colors of these points

W = size(XYZ, 2);
H = size(XYZ, 1);

whitebg('black'); % improve contrast

XYZ    = reshape(XYZ,    [W*H, size(XYZ,3)]);
colors = reshape(colors, [W*H, size(colors,3)]);

scatter3(XYZ(:,1), XYZ(:,2), XYZ(:,3), 1, colors, 'Marker', '.');

xlabel('X'); ylabel('Y'); zlabel('Z');
pbaspect([1 1 1]); % keep aspect ratio fixed
% ensure uniform scaling of all axes
set_axis_limits(XYZ);

end