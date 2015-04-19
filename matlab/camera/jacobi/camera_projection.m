function [x, D] = camera_projection( point )
% projection of point onto camera sensor: x = [u,v] = pi(p = [x,y,z])

global_parameters;

U = point(1).*CAMERA_FOCAL./point(3);
V = point(2).*CAMERA_FOCAL./point(3);
D = point(3);

% convert points on camera sensor into pixel coordinates
% int32 already rounds correctly
U = U / CAMERA_WIDTH * W + 0.5 * W;
V = V / CAMERA_WIDTH * W + 0.5 * H;

x = [U V];

end

