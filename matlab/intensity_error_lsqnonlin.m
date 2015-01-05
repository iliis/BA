function [err, J] = intensity_error_lsqnonlin( D1, I1, D2, I2, T_translation, T_rotation, plot )
% input:  2 intensitiy and 2 depth images, T transformation that maps I2/D2 to I1/D1
% output: linear error vector (not summed and not squared)

warped = warp_image(D1, I1, T_translation, T_rotation);

errs = (I2-warped);
%errs(isnan(warped)) = 0; % ignore empty pixels
errs(isnan(warped)) = 0.03; % not yet tested, penalty for unmatched pixels

err = image_to_list(errs)';

disp(['intensity error: ', num2str(T_translation), ' --> ', num2str(sum(err.^2))]);

end

