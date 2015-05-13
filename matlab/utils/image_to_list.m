function list = image_to_list(array)
% input H x W x D = 2 dimensional array of (D-dimensional) points
% output N x D list of points (N = H*W)

list = reshape(array,[],size(array,3));

end