global_parameters

image_path  = 'input';

D1 = read_depth_image(image_path, 1);
I1 = read_intensity_image(image_path, 1);
C1 = read_color_image(image_path, 1);

D2 = read_depth_image(image_path, 2);
I2 = read_intensity_image(image_path, 2);
C2 = read_color_image(image_path, 2);

scale = 1;
if scale > 1
    D1 = imresize(D1, 1/2^scale);
    I1 = imresize(I1, 1/2^scale);
    D2 = imresize(D2, 1/2^scale);
    I2 = imresize(I2, 1/2^scale);
end

ground_truth_trajectory = normalize_trajectory(read_camera_trajectory(image_path));

ground_truth = ground_truth_trajectory(2,:)-ground_truth_trajectory(1,:);
ground_truth_translation = ground_truth(1:3);
ground_truth_rotation    = ground_truth(4:6);