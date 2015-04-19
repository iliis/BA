function image = plot_bilinear( image, x, C )
% image(x = [u,v]) = C

    b = mod(x(2),1); r = mod(x(1),1);

    image = add_nan(image, floor(x(1)), floor(x(2)), (1-b)*(1-r)*C);
    image = add_nan(image, ceil (x(1)), floor(x(2)),    b *(1-r)*C);
    image = add_nan(image, floor(x(1)), ceil (x(2)), (1-b)*   r *C);
    image = add_nan(image, ceil (x(1)), ceil (x(2)),    b *   r *C);

end


function img = add_nan(img, u, v, C)

    if (isnan(img(v,u)))
        img(v,u) = 0;
    end

    img(v,u) = img(v,u) + C;

end