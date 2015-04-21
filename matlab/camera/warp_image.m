function [warped, J] = warp_image(depth, colors, T, ignore_depth)

%disp(['warping image with pos = [' num2str(position) '] and rotation = [' num2str(rotation) ']']);

global_parameters

if nargin <= 3
    ignore_depth = false;
end

calc_jacobi = nargout > 1;
J = 0;
    

warped_color = nan(size(colors));
warped_depth = inf(size(colors));

[U,V] = meshgrid(1:W, 1:H);
UV = [reshape(U,H*W,1), reshape(V,H*W,1)];
points = camera_transformation(UV, image_to_list(depth), T);
[xs, Ds] = camera_projection(points);


% create a full depth buffer

for i = 1:size(UV,1)
    u = UV(i,1); v = UV(i,2);

    %p = camera_transformation([u,v], depth(v,u), T);
    x = xs(i, :);
    D = Ds(i);

    if ~is_in_img_range(x)
        % pixel projects to outside of image array
        continue;
    end

    if ~ignore_depth && D >= warped_depth(uint32(x(2)), uint32(x(1)))
        % pixel is behind existing one
        continue;
    end

    % actually 'plot' pixel
    warped_color(uint32(x(2)), uint32(x(1))) = colors(v,u);

    % bilinear filtering
    % doesn't give very good results tough :(
    %warped_color = plot_bilinear(warped_color, x, colors(v,u));

    warped_depth(uint32(x(2)), uint32(x(1))) = D;
end

warped = warped_color;

%XYZ = project_to_space(depth);
%XYZ = apply_camera_transformation(XYZ, T);
%warped = project_to_camera(XYZ, colors);

end