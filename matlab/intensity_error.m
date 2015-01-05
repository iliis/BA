function err = intensity_error( D1, I1, D2, I2, T_translation, T_rotation, plot )
% input:  2 intensitiy and 2 depth images, T transformation that maps I2/D2 to I1/D1
% output: sum of squared errors (differences between I2 and warped I1)

global_parameters

warped = warp_image(D1, I1, T_translation, T_rotation);

errs = (I2-warped).^2;
%errs(isnan(warped)) = 0; % ignore empty pixels
errs(isnan(warped)) = NONMATCHED_PIXEL_PENALTY; % empiric penalty for empty pixels

err = sum(sum(errs));


if nargin >= 7
    if numel(plot) ~= 3
        plot = [subplot(1,3,1) subplot(1,3,2) subplot(1,3,3)];
    end
    
    axes(plot(1));
    imagesc(I2);

    axes(plot(2));
    imagesc(warped);

    axes(plot(3));
    imagesc(abs(errs));
    
    title(err);
end

end