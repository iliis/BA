function I = read_intensity_image( path, nr, intrinsics )

I = color_to_intensity(read_color_image(path, nr, intrinsics));

end

