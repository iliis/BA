I = imread('color0001.png'); % RGB, 8bit / channel
D = double(imread('depth0001.png')); % BW, 16bit

% properties of camera (in Blender Units / meter)
focal = 0.035; % 35mm
width = 0.032; % sensor size = 32mm
clipping = [0.1 100];

% initial position of camera
camera_pos = [ 7.5 -6.5 5.3 ];
%camera_rot = [ 0.782 0.482 0.213 0.334 ];
camera_rot = [0.7816000580787659, 0.48170700669288635, 0.21292176842689514, 0.3342514932155609];


% calculate camera rotation matrix
T = quat2dcm(camera_rot)';

% scale depth
D = D / 65535 * (clipping(2)-clipping(1)) + clipping(1);

center = size(D)/2;
[U,V] = meshgrid((1:size(D,2)) - center(2), (1:size(D,1)) - center(1));
U = U * width / size(D,2); % correct for sensor size
V = V * width / size(D,2);

% Blender's camera has Y as 'up' and -Z as 'into the scene'
pointsx =  U.*D;
pointsy = -V.*D;
pointsz = -focal.*D;

pointsx = reshape(pointsx, [1,numel(pointsx)]);
pointsy = reshape(pointsy, [1,numel(pointsy)]);
pointsz = reshape(pointsz, [1,numel(pointsz)]);

% apply camera transformation
points = T * [pointsx; pointsy; pointsz]; % + camera_pos;

c = numel(pointsx);
colors = double([reshape(I(:,:,1), [c,1]) reshape(I(:,:,2), [c,1]) reshape(I(:,:,3), [c,1])]) ./ 255;

whitebg('black'); % improve contrast
scatter3(points(1,:), points(2,:), points(3,:), 1, colors, 'Marker', '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
pbaspect([1 1 1]); % keep aspect ratio fixed
% ensure uniform scaling of all axes
xlim([-1 0]);
ylim([ 0 1]);
zlim([-0.5 0.5]);