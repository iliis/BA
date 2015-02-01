function plot_difference(I1, I2)
errs = (I1 - I2).^2;
errs(isnan(I2)) = [];
err = sum(sum(errs));

subplot(1,3,1);
imagesc(I1);
title('Image 1');

subplot(1,3,2);
imagesc(I2);
title('Image 2');

subplot(1,3,3);
diff = abs(I1 - I2);
diff(isnan(I2)) = 0;
imagesc(diff);
title(['diff (err = ', num2str(err), ')']);
end