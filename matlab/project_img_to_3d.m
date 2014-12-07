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

whitebg('black'); % improve contrast
scatter3(XYZ(:,1), XYZ(:,2), XYZ(:,3), 1, colors, 'Marker', '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
pbaspect([1 1 1]); % keep aspect ratio fixed
% ensure uniform scaling of all axes
xlim([-1 0]);
ylim([ 0 1]);
zlim([-0.5 0.5]);