function XYZ = project_to_space(D)
% input:  H x W image with depth values
% output: H x W x 3 list of points in real space

global_parameters

H = size(D, 1);
W = size(D, 2);

% calculate pixel positions on camera sensor
[U,V] = meshgrid((1:W) - W/2, (1:H) - H/2);
U = U * (CAMERA_WIDTH / W); % correct for sensor size
V = V * (CAMERA_WIDTH / W);

% actually project points into 3D space
X = U .* D / CAMERA_FOCAL;
Y = V .* D / CAMERA_FOCAL;
Z = D;

XYZ = cat(3, X, Y, Z);

end