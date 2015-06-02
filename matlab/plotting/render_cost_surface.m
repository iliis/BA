function [ errs, gradients ] = render_cost_surface( scene, dimensions, ranges, T, render_gradient )
% renders the cost surface for given area and dimensions
%
% INPUT:
%
% scene
%   Scene, containing a pair of images
%
% dimensions
%   [ dim1 dim2 ]: along which dimensions to render
%
% ranges
%   { linspace(...) linspace(...) } sample points for dim1/dim2
%
% T = [0 0 0 0 0 0]'
%   Transformation values for other dimensions
%
% render_gradient = false
%   if true, Jacobians are evaluated and plotted as well

% INPUT PARAMETER VERIFICATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

assert(isa(scene, 'Scene'));

assert(numel(dimensions) == 2);
assert(all(dimensions >= 1), 'dimensions must contain valid index into 6 dimensional transformation vector');
assert(all(dimensions <= 6), 'dimensions must contain valid index into 6 dimensional transformation vector');
assert(dimensions(1) ~= dimensions(2), 'dimensions cannot be the same, otherwise we would get 1 dim plot');

assert(isa(ranges, 'cell'));
assert(numel(ranges) == 2);

if nargin < 4
    T = [0 0 0 0 0 0]';
end

assert(size(T,1) == 6);
assert(size(T,2) == 1);

if nargin < 5
    render_gradient = false;
end

if nargout > 1 && ~render_gradient
    warning('render_gradient is false, no gradient will be calculated!');
end

% DEFINES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dim_names = {'X', 'Y', 'Z', 'alpha', 'beta', 'gamma'};

% CALCULATE ERROR SURFACE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Nx = numel(ranges{1});
Ny = numel(ranges{2});

errs  = zeros(Ny, Nx);
gradients = zeros(Ny, Nx, 6);
progress(0, [0 Ny]); % show progress bar from the start
for y = 1:Ny
    for x = 1:Nx
        v = T;
        v(dimensions(1)) = ranges{1}(x);
        v(dimensions(2)) = ranges{2}(y);
        
        % actual error calculation
        if render_gradient
            [e, J] = camera_warp(scene, v);
            gradients(y,x,:) = - J' * e';
        else
            e = camera_warp(scene, v);
        end
        
        v
        errs(y,x) = norm(e);
        errs(y,x)
    end
    
    % progress() isn't thread-safe :(
    progress(y, [0 Ny]);
end

[miny, iy] = min(errs);
[minx, ix] = min(miny);
minpos_x = ranges{1}(ix);
minpos_y = ranges{2}(iy(ix));

% PLOT ERROR SURFACE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

imagesc(minmax(ranges{1}),minmax(ranges{2}),errs);
hold on;
plot(minpos_x, minpos_y, 'or');
plot(scene.ground_truth(dimensions(1)), scene.ground_truth(dimensions(2)), 'xg');

title({['Error (min = ' num2str(min(min(errs))) ')'], ...
     ['min at ' dim_names{dimensions(1)} ' = ' num2str(minpos_x) ', ' dim_names{dimensions(2)} ' = ' num2str(minpos_y)], ...
     ['solution at ' dim_names{dimensions(1)} ' = ' num2str(scene.ground_truth(dimensions(1))) ...
                ', ' dim_names{dimensions(2)} ' = ' num2str(scene.ground_truth(dimensions(2)))], ...
     ... %['scale = ' num2str(image_scale)], ...
     ['image = ' scene.source_path ]}, ...
     'Interpreter','none'); % don't treat underscores as control characters
xlabel(dim_names{dimensions(1)});
ylabel(dim_names{dimensions(2)});
colorbar;

if render_gradient
    quiver(ranges{1}, ranges{2}, gradients(:,:,dimensions(1)), gradients(:,:,dimensions(2)), 'r');
end

hold off;

end

