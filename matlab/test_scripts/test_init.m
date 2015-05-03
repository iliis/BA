global_parameters

if ~exist('image_path', 'var') || isempty(image_path)
    image_path  = 'input';
end

global image_scale;
if ~exist('image_scale', 'var') || isempty(image_scale) || image_scale < 1
    image_scale = 1;
end

D1 = read_depth_image(image_path, 1);
I1 = read_intensity_image(image_path, 1);
C1 = read_color_image(image_path, 1);

D2 = read_depth_image(image_path, 2);
I2 = read_intensity_image(image_path, 2);
C2 = read_color_image(image_path, 2);

if image_scale > 1
    D1 = imresize(D1, 1/2^(image_scale-1));
    I1 = imresize(I1, 1/2^(image_scale-1));
    C1 = imresize(C1, 1/2^(image_scale-1));
    
    D2 = imresize(D2, 1/2^(image_scale-1));
    I2 = imresize(I2, 1/2^(image_scale-1));
    C2 = imresize(C2, 1/2^(image_scale-1));
end

ground_truth_trajectory = read_camera_trajectory(image_path);

ground_truth = ground_truth_trajectory(2,:);
ground_truth_translation = ground_truth(1:3);
ground_truth_rotation    = ground_truth(4:6);