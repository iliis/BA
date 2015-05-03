dim1 = 1;
dim2 = 2;
dim_names = {'X', 'Y', 'Z', 'alpha', 'beta', 'gamma'};



global image_scale;
image_scale = 4;
%image_path = 'input';
%image_path = 'unit_tests/rot_trans_verysmall';
image_path = 'unit_tests/linear_translation';
test_init;

minfun = @(x) intensity_error(D1,I1,I2, x);

%xrange = linspace(-10,10,40);
%yrange = linspace(-10,10,40);

xrange = linspace(0.7,1.5,100);
yrange = linspace(-0.3,0.3,40);

%xrange = linspace(-0.6, 0.1, 100);
%yrange = linspace(-0.2, 0.2, 70);

Nx = numel(xrange);
Ny = numel(yrange);

errs  = zeros(Ny, Nx);
steps = zeros(Ny, Nx, 6);
for y = 1:Ny
    for x = 1:Nx
        v = [0 0 0 0 0 0];
        v(dim1) = xrange(x);
        v(dim2) = yrange(y);
        [e, J] = minfun(v);
        %e = minfun(v);
        errs(y,x) = sum(e.^2);
        
        steps(y,x,:) = - J' * e;
    end
    
    % progress() isn't thread-safe :(
    progress(y, [0, numel(yrange)]);
end

[miny, iy] = min(errs);
[minx, ix] = min(miny);
minpos_x = xrange(ix);
minpos_y = yrange(iy(ix));

%subplot(1,2,1);
imagesc(minmax(xrange),minmax(yrange),errs);
hold on;
plot(minpos_x, minpos_y, 'or');
plot(ground_truth(dim1), ground_truth(dim2), 'xg');
hold off;
title({['Error (min = ' num2str(min(min(errs))) ')'], ...
    ['min at ' dim_names{dim1} ' = ' num2str(minpos_x) ', ' dim_names{dim2} ' = ' num2str(minpos_y)], ...
    ['solution at ' dim_names{dim1} ' = ' num2str(ground_truth(dim1)) ', ' dim_names{dim2} ' = ' num2str(ground_truth(dim2))], ...
    ['scale = ' num2str(image_scale)], ...
    ['image = ' image_path ]}, ...
    'Interpreter','none'); % don't treat underscores as control characters
xlabel(dim_names{dim1});
ylabel(dim_names{dim2});
colorbar;

%figure;
hold on;
quiver(xrange, yrange, steps(:,:,dim1), steps(:,:,dim2), 'r');

%subplot(1,2,2);
%quiver(diff1, diff2);