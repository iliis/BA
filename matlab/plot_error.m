
D1 = read_depth_image(1);
I1 = read_intensity_image(1);
D2 = read_depth_image(2);
I2 = read_intensity_image(2);

% [-2.1650    0.3075   -0.4952]; % actual solution


minfunpos  = @(x) intensity_error(D1,I1,D2,I2, x(1:3), [0 0 0], false);

xrange = -10:0.2:10;
yrange = -10:0.2:10;

errs = zeros(numel(yrange), numel(xrange));
for y = 1:numel(yrange)
    for x = 1:numel(xrange)
        errs(y,x) = minfunpos([xrange(x) yrange(y) -0.5]);
    end
    
    progress(y, [0, numel(yrange)]);
end

imagesc(errs);