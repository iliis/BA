function plot_pointcloud(XYZ, colors)

whitebg('black'); % improve contrast

scatter3(XYZ(:,1), XYZ(:,2), -XYZ(:,3), 1, colors, 'Marker', '.');

xlabel('X'); ylabel('Y'); zlabel('Z');
pbaspect([1 1 1]); % keep aspect ratio fixed
% ensure uniform scaling of all axes
set_axis_limits(XYZ);

end