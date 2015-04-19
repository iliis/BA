function I = color_to_intensity(RGB)
% input:  H x W x 3 image with RGB values (0-1)
% output: H x W     image of intensity values, also in range 0-1

% L2 norm
%I = sqrt(sum(RGB.^2,3));

% L1 norm, normalized to 0-1
I = sum(RGB,3)./size(RGB,3);

end