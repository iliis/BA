function J_I = jacobi_image(I,x)
% Jacobian for Image intensitiy itself

% TODO: filter/convolute image?
%diff_x = diff(I,1,1);
%diff_y = diff(I,1,2);


U = x(:,1);
V = x(:,2);

diff_x = conv2(I, [-1 0 1]/2, 'same');
diff_y = conv2(I, [-1 0 1]'/2, 'same');

% approximate Jacobian for intensity image

J_I = zeros(numel(I),2);

% only assign a value when the pixel actually lies inside the image
% otherwise, J = [0 0]
valid = is_in_img_range(x);
J_I(valid, :) = [ diff_x(sub2ind(size(diff_x), V(valid), U(valid))), diff_y(sub2ind(size(diff_y), V(valid), U(valid))) ];

% we want a 1x2xN matrix instead of Nx2
J_I = permute(J_I, [3 2 1]);

end

