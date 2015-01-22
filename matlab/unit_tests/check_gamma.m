raw_depth = imread('depth_gamma_test0001.png'); % BW, 16bit

% properties of camera (in Blender Units / meter)
clipping = [5 10];

% scale depth
scaled_depth = double(raw_depth) / DEPTH_IMAGE_MAXVAL * (clipping(2)-clipping(1)) + clipping(1);

%plot(raw_depth(size(raw_depth,1)/2,:));
plot(scaled_depth(size(raw_depth,1)/2,:));

%corrected_depth = (double(raw_depth) / 65535) .^ 2.2;
%plot(corrected_depth(size(raw_depth,1)/2,:));