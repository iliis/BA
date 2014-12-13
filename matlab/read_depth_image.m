function D = read_depth_image(nr)
% output: N x 3 coordinates (U, V are in camera plane, D is distance from
% camera origin)

% properties of camera (in Blender Units / meter)
clipping = [0.1 100];

D = imread(sprintf('depth%04u.png', nr)); % BW, 16bit

% reverse gamma correction
%D = imadjust(D,[],[],2.2);

% scale depth
D = double(D) / 65535 * (clipping(2)-clipping(1)) + clipping(1);

end