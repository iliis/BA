function [UVD] = read_depth_image(nr)
% output: N x 3 coordinates (U, V are in camera plane, D is distance from
% camera origin)

% properties of camera (in Blender Units / meter)
clipping = [0.1 100];
width = 0.032; % sensor size = 32mm

D = imread(sprintf('depth%04u.png', nr)); % BW, 16bit

% reverse gamma correction
%D = imadjust(D,[],[],2.2);

% scale depth
D = double(D) / 65535 * (clipping(2)-clipping(1)) + clipping(1);

center = size(D)/2;
[U,V] = meshgrid((1:size(D,2)) - center(2), (1:size(D,1)) - center(1));
U = U * width / size(D,2); % correct for sensor size
V = V * width / size(D,2);

UVD = [reshape(U, [numel(U),1]), reshape(V, [numel(V),1]), reshape(D, [numel(D),1])];

end