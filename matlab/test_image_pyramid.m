clear all;
global_parameters;

MAX_ITERS = 5; % how often to divide images by 2

% initial estimation
T_translation = [ 0 0 0 ]; % actual solution = 2.1411    0.3076   -0.5665
T_rotation    = [ 0 0 0 ]; % not yet used!

D1 = read_depth_image(1);
I1 = read_intensity_image(1);
D2 = read_depth_image(2);
I2 = read_intensity_image(2);

options = optimset();
%options = optimset(options, 'TolX', 1e-10, 'TolFun', 1e-10, 'TolCon', 1e-10);
options = optimset(options, 'TolX', 0.0001, 'TolFun', 1);
options = optimset(options, 'DiffMinChange', 0.01);
options = optimset(options, 'DiffMaxChange', 1);
options = optimset(options, 'Display', 'iter-detailed', 'FunValCheck', 'on');
options = optimset(options, 'UseParallel', true);
%options = optimset(options, 'Jacobian', 'on');


for i = MAX_ITERS:-1:1
    
    D1_scaled = imresize(D1, 1/2^i);
    I1_scaled = imresize(I1, 1/2^i);
    D2_scaled = imresize(D2, 1/2^i);
    I2_scaled = imresize(I2, 1/2^i);
    
    minfunl    = @(x) intensity_error_lsqnonlin(D1_scaled,I1_scaled,I2_scaled, x(1:3), x(4:6));
    minfun     = @(x) intensity_error(D1_scaled,I1_scaled,D2_scaled,I2_scaled, x(1:3), x(4:6));

    %[T_translation, ymin] = lsqnonlin(minfunposl, T_translation, [-3 -2 -2], [2 2 2], options)
    %[T_translation, ymin] = patternsearch(minfunpos, T_translation)
    %xmin = patternsearch(minfun, [T_translation T_rotation])
    xmin = lsqnonlin(minfunl, [T_translation T_rotation], [-3 -2 -2 -2 -2 -2], [2 2 2 2 2 2], options);
    T_translation = xmin(1:3);
    T_rotation    = xmin(4:6);

    %intensity_error(D1,I1,D2,I2, T_translation, T_rotation, true); % plot result
    
    subplots = [subplot(MAX_ITERS,3,i*3-2) subplot(MAX_ITERS,3,i*3-1) subplot(MAX_ITERS,3,i*3)];
    intensity_error(D1_scaled,I1_scaled,D2_scaled,I2_scaled, T_translation, T_rotation, subplots ); % plot result
    drawnow; % force update of plot
end

points1 = apply_camera_transformation(project_to_space(D1), T_translation, T_rotation);
points2 = project_to_space(D2);

write_to_ply([points1 points2], [read_color_image(1) read_color_image(2)], 'test.ply');

% result:  -2.1794    0.2944   -0.4411
% true:     2.1411    0.3076   -0.5665