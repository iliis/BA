function inside = is_in_img_range( x )
% returns true if x = [u,v] is inside image dimensions (1:W, 1:H)

global_parameters;

inside = floor(x(1)) >= 1 && ceil(x(1)) <= W && floor(x(2)) >= 1 && ceil(x(2)) <= H;

end

