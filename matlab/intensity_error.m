function err = intensity_error( D1, I1, D2, I2, T_translation, T_rotation, plot )
% input 2 intensitiy and 2 depth images, T transformation that maps I2/D2
% to I1/D1

if (nargin < 7)
    plot = false;
end

warped = warp_image(D1, I1, T_translation, T_rotation);

errs = (I2-warped).^2;
errs(isnan(warped)) = 0; % ignore empty pixels

err = sum(sum(errs));


if (plot)
    subplot(3,1,1);
    imagesc(I2);
    subplot(3,1,2);
    imagesc(warped);
    subplot(3,1,3);
    imagesc(abs(errs));
    title(err);
end

end

