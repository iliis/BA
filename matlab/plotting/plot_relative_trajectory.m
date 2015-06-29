function points = plot_relative_trajectory( path, in3d, varargin )
% plots a 3D trajectory from list of relative transformations

% INPUT
%
% path
%   6 x N list of transformations [x y z alpha beta gamma]'

N = size(path, 2);
assert(size(path, 1) == 6);

points = zeros(3, N+1);

current_pos = [0 0 0]';
current_rot = eye(3);

current_rot = angle2dcm([deg2rad(0) deg2rad(0) deg2rad(0)]);

for i = 1:N
    
    abs_delta = current_rot * path(1:3,i);
    
    current_pos = current_pos + abs_delta;
    current_rot = angle2dcm(path(4:6,i)) * current_rot;
    
    points(:, i+1) = current_pos;
end

if in3d
    plot3(points(1,:), points(3,:), points(2,:), varargin{:});
    xlabel('X [m]');
    ylabel('Z [m]');
    zlabel('Y [m]');
    %set(gca, 'Zdir', 'reverse');
else
    plot(points(1,:), points(3,:), varargin{:});
    xlabel('X [m]');
    ylabel('Z [m]');
end

%hold on;
%plot3(points(1,:), points(2,:), zeros(size(points,2),1), varargin{:});
%hold off;

%set(gca, 'DataAspectRatio', [1 1 1]);

box on;
grid on;

end

