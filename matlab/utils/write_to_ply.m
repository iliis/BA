function write_to_ply(points, colors, output_filename)
% writes a H x W x ? image into a .ply file

file = fopen(output_filename, 'w');

points = image_to_list(points);
colors = image_to_list(colors);

% write header
fprintf(file, 'ply\nformat ascii 1.0\nelement vertex %d\n', size(points,2));

fprintf(file, 'property float x\n');
fprintf(file, 'property float y\n');
fprintf(file, 'property float z\n');
fprintf(file, 'property uchar red\n');
fprintf(file, 'property uchar green\n');
fprintf(file, 'property uchar blue\n');

fprintf(file, 'end_header\n');

for i = 1:size(points,2)
    fprintf(file, '%f %f %f ',   points(:,i));
    fprintf(file, '%d %d %d\n', colors(:,i)*255);
    
    if (mod(i,10000) == 0)
        progress(i,[1,size(points,2)]);
    end
end
progress(size(points,2),[1,size(points,2)]); % close progess bar

fclose(file);

end