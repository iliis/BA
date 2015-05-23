function do_global_minimization( image_path, image_scale, min_method, restrict_to_xy, step_size, T_init)

D1 = read_depth_image(image_path, 1);
I1 = read_intensity_image(image_path, 1);
%C1 = read_color_image(image_path, 1);

%D2 = read_depth_image(image_path, 2);
I2 = read_intensity_image(image_path, 2);
%C2 = read_color_image(image_path, 2);

if image_scale > 1
    D1 = imresize(D1, 1/2^(image_scale-1));
    I1 = imresize(I1, 1/2^(image_scale-1));
    %C1 = imresize(C1, 1/2^(image_scale-1));
    
    %D2 = imresize(D2, 1/2^(image_scale-1));
    I2 = imresize(I2, 1/2^(image_scale-1));
    %C2 = imresize(C2, 1/2^(image_scale-1));
end

%T = gauss_newton(D1,I1,I2, T_init);
%T = levenberg_marquardt(D1,I1,I2, T_init);
%T = gradient_descent(D1,I1,I2, T_init, 0.001);

% TODO: read intrinsics from input data
% Blender uses 35mm focal length and a 32mm wide sensor
FOCAL = 0.035 * size(I1,2) / 0.032;
intrinsics = CameraIntrinsics(size(I1,2), size(I1,1), FOCAL);

min_method(D1,I1,I2,T_init,intrinsics,restrict_to_xy,step_size);

end

