function XYZ = project_to_space(D)
% input:  H x W image with depth values
% output: H x W x 3 list of points in real space

% properties of camera (in Blender Units / meters)
focal = 0.035; % 35mm
width = 0.032; % sensor size = 32mm

H = size(D, 1);
W = size(D, 2);

% calculate pixel positions on camera sensor
[U,V] = meshgrid((1:W) - W/2, (1:H) - H/2);
U = U * (width / W); % correct for sensor size
V = V * (width / W);

% actually project points into 3D space
X = U .* D / focal;
Y = V .* D / focal;
Z = D;

XYZ = cat(3, X, Y, Z);

end