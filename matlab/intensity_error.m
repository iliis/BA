function [err, J, invalid_terms] = intensity_error( D1, I1, I2, T, plot)
% input:  2 intensitiy and 1 depth images, T transformation that maps I2/D2 to I1/D1
% output: linear error vector (not summed and not squared)
%         Jacobian of error term for T_translation

global_parameters

if nargout > 1
    [warped, J] = warp_image(D1, I1, T, false);
else
    warped = warp_image(D1, I1, T, false);
end

errs = (warped-I2);
errs(isnan(warped)) = 0; % ignore empty pixels
%errs(isnan(warped)) = NONMATCHED_PIXEL_PENALTY; % not yet tested, penalty for unmatched pixels

err = image_to_list(errs)';

% keep a copy for plotting later on
errs_plot = errs.^2;
errs_plot(isnan(warped)) = 0;
err_total = sum(err.^2);
%disp(['err_total = ' num2str(err_total) ' T = ' num2str(T)]);

invalid_terms = false(size(err));
invalid_terms(isnan(warped)) = true;

%disp(['intensity error:  --> ', num2str(sum(err.^2))]);
%disp(['translation: ' num2str(T_translation)]);
%disp(['rotation:    ' num2str(T_rotation)]);

if nargin >= 5
    if numel(plot) ~= 3
        plot = [subplot(1,3,1) subplot(1,3,2) subplot(1,3,3)];
    end
    
    axes(plot(1));
    imagesc(I2);
    title('I2');

    axes(plot(2));
    imagesc(warped);
    title('I1 warped');

    axes(plot(3));
    imagesc(abs(errs_plot));
    title(err_total);
end

end

