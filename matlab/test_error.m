clear all;



% initial position of camera
camera_pos = -[ 7.5 -6.5 5.3 ];
%camera_rot = [ 0.782 0.482 0.213 0.334 ];
camera_rot = [0.7816000580787659, 0.48170700669288635, 0.21292176842689514, 0.3342514932155609];


D1 = read_depth_image(1);
I1 = read_intensity_image(1);
D2 = read_depth_image(2);
I2 = read_intensity_image(2);

%imagesc(I2-I1);
%colormap(gray);

%err = intensity_error(D1,I1,D2,I2, [0 0 0], [1 0 0 0])

% find transformation by global optimization
%addpath('bayesopt/matlab');

minfun     = @(x) intensity_error(D1,I1,D2,I2, x(1:3), x(4:6));
minfunpos  = @(x) intensity_error(D1,I1,D2,I2, x(1:3), [0 0 0], false);
minfunposl = @(x) intensity_error_lsqnonlin(D1,I1,D2,I2, x(1:3), [0 0 0]);

guess_translation = [-2.2 0.25 -0.2];
%guess_translation = [-2.1650    0.3075   -0.4952]; % actual solution
guess_rotation    = [0 0 0];
%xmin = [guess_translation guess_rotation];

params.n_iterations = 500;
params.noise = 1e-10;
%[xmin, ymin] = bayesoptcont(minfunpos, 3, params, [-1.5,-1],[0.2,0.8],[-0.2,0.2])
%[xmin, ymin] = patternsearch(minfun, [guess_translation guess_rotation])
%[xmin, ymin] = patternsearch(minfunpos, guess_translation)

options = optimset('TolX', 1e-10, 'TolFun', 1e-10, 'TolCon', 1e-10);
options = optimset(options, 'DiffMinChange', 100, 'Display', 'iter-detailed', 'FunValCheck', 'on');
%[xmin, ymin] = lsqnonlin(minfunposl, guess_translation, [-3 -2 -2], [2 2 2], options)




intensity_error(D1,I1,D2,I2, xmin(1:3), xmin(4:6), true); % plot result



points1 = apply_camera_transformation(project_to_space(D1), xmin(1:3), xmin(4:6));
points2 = project_to_space(D2);

%write_to_ply([points1 points2], [read_color_image(1) read_color_image(2)], 'test.ply');



%intensity_error(D1,I1,D2,I2, [-1.2 0.44 0], [1 0 0 0], true);

%minfunpos([-1.2 0.44 0]);
%minfunpos([0 0 0.3374]);


%[x, fval] = patternsearch(minfun,[0 0 0 1 0 0 0])