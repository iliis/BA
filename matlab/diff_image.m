function [Dx, Dy] = diff_image(image)

assert(isequal(size(image,3), 1));

% TODO: use matrices instead of loops
Dx = zeros(size(image));
Dy = zeros(size(image));
for y = 1:size(image,1)
    for x = 1:size(image,2)
        
        Dx(y, x) = disc_diff(image, [y, x-1; y, x; y, x+1]);
        Dy(y, x) = disc_diff(image, [y-1, x; y, x; y+1, x]);
        
    end
end

end

function d = disc_diff(image, points)

% ensure we have valid points
values = [];
for i = 1:size(points,1)
    % valid index? (filter out points just outside the edge
    if (points(i,1) < 1 || points(i,1) > size(image,1)); continue; end
    if (points(i,2) < 1 || points(i,2) > size(image,2)); continue; end
    
    % is the pixel there valid?
    value = image(points(i,1), points(i,2));
    if (isnan(value) || isinf(value))
        continue
    end
    
    % passed all tests, use this pixel in gradient calculation
    values = [values value];
end

if (numel(values) <= 1)
    d = 0;
else
    d = sum(diff(values)) / (numel(values)-1);
end

end