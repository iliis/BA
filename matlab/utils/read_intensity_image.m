function I = read_intensity_image( path, nr )

I = color_to_intensity(read_color_image(path, nr));

end

