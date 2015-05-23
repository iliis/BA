function list = image_to_list(array)
% input H x W x D = 2 dimensional array of (D-dimensional) points
% output D x N list of points (N = H*W)

list = reshape(permute(array,[3,1,2]), size(array,3), []);

end