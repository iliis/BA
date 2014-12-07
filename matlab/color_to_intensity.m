function I = color_to_intensity(RGB)
% input:  N x 3 list of RGB values (0-1)
% output: N x 1 list of intensity values, also in range 0-1

% L2 norm
%I = sqrt(sum(RGB.^2,2));

% L1 norm, normalized to 0-1
I = sum(RGB,2)./size(RGB,2);

end