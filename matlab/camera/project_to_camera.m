function [image_c, image_d] = project_to_camera(XYZ, colors)
% XYZ:    H x W x 3 array of points in 3D space
% colors: H x W x ? array of colors of these points

global_parameters

H = size(XYZ, 1);
W = size(XYZ, 2);

U = XYZ(:,:,1).*CAMERA_FOCAL./XYZ(:,:,3);
V = XYZ(:,:,2).*CAMERA_FOCAL./XYZ(:,:,3);
D = XYZ(:,:,3);

% convert points on camera sensor into pixel coordinates
% int32 already rounds correctly
U = int32(U / CAMERA_WIDTH * W + 0.5 * W);
V = int32(V / CAMERA_WIDTH * W + 0.5 * H);

%plot_pointcloud(cat(3,U,V,ones(H,W)), colors);
%scatter(reshape(U,[],1), reshape(V,[],1), 'Marker', '.');

image_c = nan(size(colors));
image_d = inf(size(XYZ, 1), size(XYZ, 2));

% TODO: don't use for loop
for i = 1:H
    for j = 1:W
        u = U(i,j);
        v = V(i,j);

        %disp([num2str(U(i,j)) ' ' num2str(V(i,j)) ' --> ' num2str(u) ' ' num2str(v)]);
        
        if (u < 1 || v < 1 || u > W || v > H)
            % pixel is out of range of our image
            continue;
        end

        if (image_d(v, u) <= D(i, j))
            % current pixel is behind an existing one
            continue;
        end

        % actually "plot" projected pixel
        image_c(v, u, :) = colors(i, j, :);
        image_d(v, u)    = D(i, j);
    end
end

%image(image_c);
end