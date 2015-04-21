% tries to find minimum using Gauss Newton

clear all;

global image_scale;
image_scale = 5;
image_path = 'unit_tests/rot_trans_verysmall';
test_init;

% initial values for solvers:
%guess_translation = [-1 0 0];
%guess_translation = [-2.2 0.25 -0.2];
guess_translation = [0 0 0];
%guess_translation = [-2.1650    0.3075   -0.4952]; % actual solution = 2.1411    0.3076   -0.5665
guess_rotation    = [0 0 0];
%xmin = [guess_translation guess_rotation];


% possible problems:
% - strange cost surface at this point
% - inv(J*J') badly conditioned -> \, cholesky, ...
% - may not converge globally

T = gauss_newton(D1,I1,I2,[0 0 0 0 0 0]);


intensity_error(D1,I1,I2,T, [subplot(2,3,1) subplot(2,3,2) subplot(2,3,3)]); % plot result

% plot in full resolution
image_scale = 1;
test_init;

intensity_error(D1,I1,I2,T, [subplot(2,3,4) subplot(2,3,5) subplot(2,3,6)]); % plot result

points1 = apply_camera_transformation(project_to_space(D1), T);
points2 = project_to_space(D2);

write_to_ply([points1 points2], [C1 C2], 'test.ply');