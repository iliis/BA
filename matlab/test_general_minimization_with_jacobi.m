z = @(x, y) 3*(1-x).^2.*exp(-(x.^2) - (y+1).^2) ... 
   - 10*(x/5 - x.^3 - y.^5).*exp(-x.^2-y.^2) ... 
   - 1/3*exp(-(x+1).^2 - y.^2) + 8;

[X, Y] = meshgrid(-3:0.1:3, -3:0.1:3);

clf;
sp1 = subplot(1,2,1);
surf(X, Y, z(X, Y));

U = zeros(size(X));
V = zeros(size(Y));
for i = 1:size(X,1)
    for j = 1:size(X,2)
        [z, J] = test_function([X(i,j) Y(i,j)]);
        U(i,j) = J(1);
        V(i,j) = J(2);
    end
end

sp2 = subplot(1,2,2);
quiver(X,Y,U,V);

guess_init = [0 -1];

test_function(guess_init)

options = optimset();
%options = optimset(options, 'TolX', 1e-10, 'TolFun', 1e-10, 'TolCon', 1e-10);
options = optimset(options, 'TolX', 0.0001, 'TolFun', 0.0001);
%options = optimset(options, 'DiffMinChange', 0.01);
%options = optimset(options, 'DiffMaxChange', 1);
options = optimset(options, 'Display', 'iter-detailed', 'FunValCheck', 'on');
%options = optimset(options, 'PlotFcns', @optimplotx);
options = optimset(options, 'Jacobian', 'on');
[xmin, ymin] = lsqnonlin(@test_function, guess_init, [], [], options)

set(sp1, 'NextPlot', 'add'); % equivalent to hold on
set(sp2, 'NextPlot', 'add');
plot3(xmin(1), xmin(2), test_function(xmin), '.r', 'MarkerSize', 50, 'parent', sp1);
plot(xmin(1), xmin(2), '.r', 'MarkerSize', 50, 'parent', sp2);




options = optimset(options, 'Display', 'none');
minpoints = [];
indexes = zeros(size(X));
distances = zeros(size(X));

for i = 1:size(X,1)
    for j = 1:size(X,2)
        x = X(i,j); y = Y(i,j);
        xmin = lsqnonlin(@test_function, [x y], [], [], options);
        
        distances(i,j) = norm(xmin - [x y]);
        
        xmin = round(xmin, 1); % round to 0.1
        
        if isempty(minpoints)
            Lia = 0;
        else
            [Lia, Locb] = ismember(xmin, minpoints, 'rows');
        end
        
        if ~Lia(1) % we found a new minimum
            minpoints = [minpoints; xmin];
            indexes(i,j) = size(minpoints,1);
        else
            indexes(i,j) = Locb(1);
        end
    end
    
    progress(i,[1 size(X,1)]);
end

% remove elements outside graph
minpoints = minpoints(minpoints(:,1) <  3, :);
minpoints = minpoints(minpoints(:,1) > -3, :);
minpoints = minpoints(minpoints(:,2) <  3, :);
minpoints = minpoints(minpoints(:,2) > -3, :);

plot(minpoints(:,1), minpoints(:,2), '.b', 'parent', sp2);

figure;
imagesc(indexes);
imagesc(distances);