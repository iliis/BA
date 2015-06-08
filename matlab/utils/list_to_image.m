function array = list_to_image(list, imsize)
% INPUT:
%   D * N data points: list of column vectors
%
% OUTPUT:
%   H * W * D data points (W*H == N)
%   two dimensional array of 3rd dim. vectors

array = reshape(list', imsize(1), imsize(2), []);

end