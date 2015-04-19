function J_I = jacobi_image(I,u,v)
% Jacobian for Image intensitiy itself

% TODO: filter/convolute image?
diff_x = diff(I,1,1);
diff_y = diff(I,1,2);

% approximate Jacobian for intensity image
J_I = [ diff_x(v, u) diff_y(v, u) ];

end

