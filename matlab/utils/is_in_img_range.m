function inside = is_in_img_range( x, image )
% returns true if x = [u,v] is inside image dimensions (1:W, 1:H)

[H, W] = size(image);

if size(x,2) == 1
    U = x(1);
    V = x(2);
else
    U = x(:,1);
    V = x(:,2);
end

inside = (floor(U) >= 1) & (ceil(U) <= W) & (floor(V) >= 1) & (ceil(V) <= H);

end

