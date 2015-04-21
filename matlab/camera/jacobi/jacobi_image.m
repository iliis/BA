function J_I = jacobi_image(I,x)
% Jacobian for Image intensitiy itself

% TODO: filter/convolute image?
%diff_x = diff(I,1,1);
%diff_y = diff(I,1,2);


diff_x = conv2(I, [-1 0 1]/2, 'same');
diff_y = conv2(I, [-1 0 1]'/2, 'same');

% approximate Jacobian for intensity image
J_I = [ diff_x(x(2), x(1)) diff_y(x(2), x(1)) ];


end

