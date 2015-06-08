function directories = find_img_source_directories()

search_directories = {'input'};
directories = {};

for n = 1:numel(search_directories)
    ds = dir(fullfile(search_directories{n}));
    for d = 1:numel(ds)
        if ds(d).isdir && is_valid_dir(fullfile(search_directories{n}, ds(d).name))
            %disp(['found dir: ' ds(d).name]);
            directories = [directories, [search_directories{n} '/' ds(d).name]];
        end
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