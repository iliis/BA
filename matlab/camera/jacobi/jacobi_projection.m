function [ J_pi ] = jacobi_projection( point )
% jacobian for transformation back onto camera sensor

global_parameters;

x = point(1);
y = point(2);
z = point(3);

J_pi = ...
   [ CAMERA_FOCAL/z,       0, -(CAMERA_FOCAL*x)/z^2; ...
           0, CAMERA_FOCAL/z, -(CAMERA_FOCAL*y)/z^2];

% and back onto pixel coordinates
J_pi = J_pi * W / CAMERA_WIDTH;

end

