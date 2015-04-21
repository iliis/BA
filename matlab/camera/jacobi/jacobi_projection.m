function [ J_pi ] = jacobi_projection( point )
% jacobian for transformation back onto camera sensor

global_parameters;

x = point(:, 1);
y = point(:, 2);
z = point(:, 3);

N = zeros(size(point,1),1);
J_pi = ...
cat(3, ...
   [ CAMERA_FOCAL./z,       N, -(CAMERA_FOCAL*x)./z.^2], ...
   [       N, CAMERA_FOCAL./z, -(CAMERA_FOCAL*y)./z.^2]);

J_pi = permute(J_pi, [3 2 1]);

% and back onto pixel coordinates
J_pi = J_pi * W / CAMERA_WIDTH;

end

