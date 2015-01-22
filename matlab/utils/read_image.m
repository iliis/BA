function [ I, D ] = read_image(path, nr, scale)
%READ_IMAGE loads depth and intensity image number 'nr' and optionally scales it down by a 2^scale

if (nargin == 1)
    scale = 1;
end

D = read_depth_image(path, nr);
I = read_intensity_image(path, nr);

if scale > 1
    D = imresize(D, 1/2^scale);
    I = imresize(I, 1/2^scale);
end

end

