function trajectory_data = read_camera_trajectory( input_path )
%READ_CAMERA_TRAJECTORY loads 'camera_trajectory.csv' and returns it as matrix

trajectory_data = csvread(fullfile(input_path, 'camera_trajectory.csv'), 1, 0);

end

