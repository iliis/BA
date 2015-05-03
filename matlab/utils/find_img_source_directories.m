function directories = find_img_source_directories()

directories = {'input'};


ds = dir(fullfile('unit_tests'));
for d = 1:numel(ds)
    if ds(d).isdir && is_valid_dir(fullfile('unit_tests', ds(d).name))
        %disp(['found dir: ' ds(d).name]);
        directories = [directories, ['unit_tests/' ds(d).name]];
    end
end

end

function valid = is_valid_dir(path)

valid = (exist(fullfile(path, 'color0001.png'), 'file') == 2) ...
     && (exist(fullfile(path, 'color0002.png'), 'file') == 2) ...
     && (exist(fullfile(path, 'depth0002.png'), 'file') == 2) ...
     && (exist(fullfile(path, 'depth0002.png'), 'file') == 2) ...
     && (exist(fullfile(path, 'camera_trajectory_relative.csv'), 'file') == 2);

end