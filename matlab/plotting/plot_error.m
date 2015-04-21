dim1 = 1;
dim2 = 2;
dim_names = {'X', 'Y', 'Z', 'alpha', 'beta', 'gamma'};



global image_scale;
image_scale = 4;
image_path = 'input';
%image_path = 'unit_tests/rot_trans_verysmall';
test_init;

minfun = @(x) intensity_error(D1,I1,I2, x);

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

%subplot(1,2,1);
imagesc(minmax(xrange),minmax(yrange),errs);
title('Error');
xlabel(dim_names(dim1));
ylabel(dim_names(dim2));
colorbar;

%subplot(1,2,2);
%quiver(diff1, diff2);