function warped = warp_image(depth, colors, T)

%disp(['warping image with pos = [' num2str(position) '] and rotation = [' num2str(rotation) ']']);

global_parameters

warped_color = nan(size(colors));
warped_depth = inf(size(colors));

for v=1:H
    for u=1:W
        p = camera_transformation([u,v], depth(v,u), T);
        [x, D] = camera_projection(p);
        
        if ~is_in_img_range(x)
            % pixel projects to outside of image array
            continue;
        end
        
        if D >= warped_depth(uint32(x(2)), uint32(x(1)))
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
end

warped = warped_color;

%XYZ = project_to_space(depth);
%XYZ = apply_camera_transformation(XYZ, T);
%warped = project_to_camera(XYZ, colors);

end