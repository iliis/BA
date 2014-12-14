function list = image_to_list(array)
% input H x W x 3 = 2 dimensional array of points
% output N x 3 list of points (N = H*W)

list = reshape(array,[],size(array,3));

end