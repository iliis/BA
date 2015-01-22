dim1 = 1;
dim2 = 2;

scale = 4;

D1 = read_depth_image(1);
I1 = read_intensity_image(1);
D2 = read_depth_image(2);
I2 = read_intensity_image(2);

if scale > 1
    D1 = imresize(D1, 1/2^scale);
    I1 = imresize(I1, 1/2^scale);
    D2 = imresize(D2, 1/2^scale);
    I2 = imresize(I2, 1/2^scale);
end

% [-2.1650    0.3075   -0.4952]; % actual solution


minfun = @(x) intensity_error_lsqnonlin(D1,I1,I2, x(1:3), x(4:6));

xrange = -10:0.5:10;
yrange = -10:0.5:10;

Nx = numel(xrange);
Ny = numel(yrange);

errs = zeros(Ny, Nx);
diff1 = zeros(Ny, Nx);
diff2 = zeros(Ny, Nx);
for y = 1:Ny
    for x = 1:Nx
        %errs(y,x) = minfunpos([xrange(x) yrange(y) 0.386]);
        v = [0 0 0.385 0 0 0];
        v(dim1) = xrange(x);
        v(dim2) = yrange(y);
        [e, J] = minfun(v);
        errs(y,x) = sum(e.^2);
        diff1(y,x) = sum(J(:,dim1)) / numel(J(:,dim1));
        diff2(y,x) = sum(J(:,dim2)) / numel(J(:,dim2));
    end
    
    % progress() isn't thread-safe :(
    progress(y, [0, numel(yrange)]);
end

subplot(1,2,1);
imagesc(errs);

subplot(1,2,2);
quiver(diff1, diff2);